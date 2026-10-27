#!/usr/bin/env python3
"""Run the OAK driver; exit 1 if it is not ready within RAMMP_OAK_READY_TIMEOUT.

When driver.i_ip / driver.i_device_id match no reachable camera, the driver
retries discovery forever ("Ignoring device info: ...") and never exits, so a
supervisor sees a healthy node that publishes nothing. This kills it once the
timeout passes without "Driver ready!" in its log, and exits non-zero so the
node shows as crashed. 0 disables the timeout.

Encoded color (RAMMP_OAK_COLOR_CODEC=h264 or h265): the camera encodes color
on-device (set color.i_low_bandwidth, i_publish_compressed and
i_low_bandwidth_profile in params: 2 for h264, or 3 with
i_low_bandwidth_ffmpeg_encoder libx265 for h265), so the link carries a small
fraction of raw.
Downstream users should not notice: the driver's packets are moved to
/<node>/color/image_raw/ffmpeg, and once the driver is ready a decoder
(image_transport republish with the ffmpeg plugin) publishes sensor_msgs/Image
on the usual /<node>/color/image_raw with the original header stamp and
frame_id. camera_info and depth are untouched. The decoder is supervised with
the driver: if either exits, both stop and the container exits (1 if the
decoder was the one that died).

The driver's output passes through unchanged. SIGTERM/SIGINT are forwarded,
since this runs as the container's PID 1.
"""
import os
import signal
import subprocess
import sys
import threading

READY = "Driver ready!"
REPUBLISH = "/opt/ros/humble/lib/image_transport/republish"
# ffmpeg_image_transport picks the decoder list by the packet's encoder label
# (param in.ffmpeg.decoders.<label>) and rejects decoders that don't match the
# label's codec. Its default list tries *_cuvid first, which segfaults in a
# container without libcuda, so the software decoder is set explicitly.
# The label is color.i_low_bandwidth_ffmpeg_encoder, which stock depthai-ros
# 3.3.0 ignores (every packet "libx264"); the fork in this image honours it, so
# h265 needs it set to libx265 (rammp-suite-software#8, #10).
DECODERS = {"h264": ("libx264", "h264"), "h265": ("libx265", "hevc")}


def remap(argv, key, default):
    """The value of the first `-r KEY:=VALUE` remap in argv (the one ROS 2 applies), else default."""
    for a in argv:
        if a.startswith(key + ":="):
            return a.split(":=", 1)[1]
    return default


def encoded_commands(argv, codec):
    """(driver argv, decoder argv, packets topic, image topic) for encoded color."""
    name = remap(argv, "__node", "driver")
    ns = remap(argv, "__ns", "").strip("/")
    base = f"/{ns}/{name}" if ns else f"/{name}"
    image, packets = f"{base}/color/image_raw", f"{base}/color/image_raw/ffmpeg"
    driver = list(argv) + ["--ros-args", "-r", f"{image}/compressed:={packets}"]
    decoder = [REPUBLISH, "ffmpeg", "raw", "--ros-args",
               "-r", f"__node:={name}_color_decoder",
               "-r", f"in/ffmpeg:={packets}", "-r", f"out:={image}",
               "-p", "in.ffmpeg.decoders.{}:={}".format(*DECODERS[codec])]
    return driver, decoder, packets, image


def main():
    timeout = float(os.environ.get("RAMMP_OAK_READY_TIMEOUT", "60"))
    codec = os.environ.get("RAMMP_OAK_COLOR_CODEC", "").lower()
    if codec and codec not in DECODERS:
        sys.exit(f"[rammp] RAMMP_OAK_COLOR_CODEC={codec}: expected one of {sorted(DECODERS)}")
    argv, decoder_argv = sys.argv[1:], None
    if codec:
        argv, decoder_argv, packets, image = encoded_commands(argv, codec)
    driver = subprocess.Popen(argv, stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT, text=True, bufsize=1)
    procs = [driver]
    ready = threading.Event()
    timed_out = threading.Event()
    stopping = threading.Event()
    decoder_died = threading.Event()

    def forward(sig, _):
        stopping.set()
        for p in procs:
            p.send_signal(sig)

    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, forward)

    def watchdog():
        if ready.wait(timeout):
            return
        print(f"[rammp] OAK driver not ready after {timeout:g}s: no camera matches "
              "driver.i_ip / driver.i_device_id, or it is unreachable. Exiting.",
              file=sys.stderr, flush=True)
        timed_out.set()
        # SIGKILL: the driver segfaults on SIGTERM while it is still searching.
        driver.kill()

    if timeout > 0:
        threading.Thread(target=watchdog, daemon=True).start()

    def stop_driver_when(decoder):
        decoder.wait()
        if driver.poll() is None and not stopping.is_set():
            decoder_died.set()
            print("[rammp] color decoder exited; stopping the driver", file=sys.stderr, flush=True)
            driver.terminate()

    for line in driver.stdout:
        sys.stdout.write(line)
        sys.stdout.flush()
        if not ready.is_set() and READY in line:
            ready.set()
            if decoder_argv:
                print(f"[rammp] {codec} color: decoding {packets} -> {image}", flush=True)
                decoder = subprocess.Popen(decoder_argv)
                procs.append(decoder)
                threading.Thread(target=stop_driver_when, args=(decoder,), daemon=True).start()

    code = driver.wait()
    for p in procs[1:]:
        if p.poll() is None:
            p.terminate()
            p.wait()
    if timed_out.is_set() or decoder_died.is_set():
        sys.exit(1)
    sys.exit(128 - code if code < 0 else code)


if __name__ == "__main__":
    main()
