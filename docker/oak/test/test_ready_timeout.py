"""Unit tests for the OAK wrapper's encoded-color command construction (no ROS needed).
Run: python3 -m pytest docker/oak/test"""
import importlib.util
import os
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
WRAPPER = os.path.join(HERE, "..", "ready_timeout.py")
spec = importlib.util.spec_from_file_location("ready_timeout", WRAPPER)
rt = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rt)

DRIVER = ["/opt/ros/humble/lib/depthai_ros_driver_v3/driver_node", "--ros-args", "--params-file",
          "/etc/rammp/oak/oak.yaml", "--ros-args", "-r", "__node:=wrist_camera"]


def test_remap():
    assert rt.remap(DRIVER, "__node", "driver") == "wrist_camera"
    assert rt.remap(["driver_node"], "__node", "driver") == "driver"
    # ROS 2 applies the first matching rule, so a later rename is ignored.
    assert rt.remap(DRIVER + ["-r", "__node:=scene_camera"], "__node", "driver") == "wrist_camera"


def test_h264_moves_packets_and_decodes_back_to_image_raw():
    driver, decoder, packets, image = rt.encoded_commands(DRIVER, "h264")
    assert driver[:len(DRIVER)] == DRIVER
    assert driver[len(DRIVER):] == ["--ros-args", "-r",
                                    "/wrist_camera/color/image_raw/compressed:=/wrist_camera/color/image_raw/ffmpeg"]
    assert (packets, image) == ("/wrist_camera/color/image_raw/ffmpeg", "/wrist_camera/color/image_raw")
    assert decoder[:3] == [rt.REPUBLISH, "ffmpeg", "raw"]
    assert "in/ffmpeg:=/wrist_camera/color/image_raw/ffmpeg" in decoder
    assert "out:=/wrist_camera/color/image_raw" in decoder
    assert "__node:=wrist_camera_color_decoder" in decoder
    assert "in.ffmpeg.decoders.libx264:=h264" in decoder  # keyed by packet label


def test_default_node_name_is_the_drivers():
    assert rt.encoded_commands(["driver_node"], "h264")[2:] == ("/driver/color/image_raw/ffmpeg",
                                                                "/driver/color/image_raw")


def test_namespace_remap_is_honoured():
    _, _, packets, image = rt.encoded_commands(DRIVER + ["-r", "__ns:=/robot"], "h264")
    assert (packets, image) == ("/robot/wrist_camera/color/image_raw/ffmpeg", "/robot/wrist_camera/color/image_raw")


def run_wrapper(*argv, **env):
    base = {k: v for k, v in os.environ.items() if k != "RAMMP_OAK_COLOR_CODEC"}
    return subprocess.run([sys.executable, WRAPPER, *argv], env={**base, **env}, capture_output=True, text=True)


def test_unknown_codec_exits_nonzero():
    r = run_wrapper("true", RAMMP_OAK_COLOR_CODEC="vp9")
    assert r.returncode != 0 and "vp9" in r.stderr


def test_h265_is_rejected_until_the_driver_labels_it():
    r = run_wrapper("true", RAMMP_OAK_COLOR_CODEC="h265")
    assert r.returncode != 0 and "issues/10" in r.stderr


def test_raw_mode_passes_exit_code_through():
    r = run_wrapper(sys.executable, "-c", "print('Driver ready!'); raise SystemExit(3)")
    assert r.returncode == 3 and "Driver ready!" in r.stdout


# A driver that, like driver_node, exits 0 on SIGTERM.
FAKE_DRIVER = ("import signal, sys, time; signal.signal(signal.SIGTERM, lambda *a: sys.exit(0)); "
               "print('Driver ready!', flush=True); time.sleep(30)")


def encoded_wrapper(decoder_code):
    """The wrapper in encoded mode, with fake driver and decoder processes."""
    code = (f"import importlib.util, sys; spec = importlib.util.spec_from_file_location('rt', {WRAPPER!r}); "
            "rt = importlib.util.module_from_spec(spec); spec.loader.exec_module(rt); "
            f"rt.encoded_commands = lambda argv, codec: ([sys.executable, '-c', {FAKE_DRIVER!r}], "
            f"[sys.executable, '-c', {decoder_code!r}], 'packets', 'image'); rt.main()")
    return subprocess.Popen([sys.executable, "-c", code], env={**os.environ, "RAMMP_OAK_COLOR_CODEC": "h264"},
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)


def test_decoder_crash_exits_nonzero():
    p = encoded_wrapper("raise SystemExit(139)")
    _, err = p.communicate(timeout=20)
    assert p.returncode == 1 and "decoder exited" in err


def test_sigterm_is_a_clean_stop_even_if_the_decoder_exits_first():
    p = encoded_wrapper("import time; time.sleep(30)")
    for line in p.stdout:
        if "decoding" in line:
            break
    time.sleep(0.5)
    p.send_signal(signal.SIGTERM)
    _, err = p.communicate(timeout=20)
    assert p.returncode == 0 and "decoder exited" not in err
