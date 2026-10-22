"""Unit tests for the OAK wrapper's encoded-color command construction (no ROS needed).
Run: python3 -m pytest docker/oak/test"""
import importlib.util
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("ready_timeout", os.path.join(HERE, "..", "ready_timeout.py"))
rt = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rt)

DRIVER = ["/opt/ros/humble/lib/depthai_ros_driver_v3/driver_node", "--ros-args", "--params-file",
          "/etc/rammp/oak/oak.yaml", "--ros-args", "-r", "__node:=wrist_camera"]


def test_node_name():
    assert rt.node_name(DRIVER) == "wrist_camera"
    assert rt.node_name(["driver_node"]) == "oak"


def test_h265_moves_packets_and_decodes_back_to_image_raw():
    driver, decoder, packets, image = rt.encoded_commands(DRIVER, "h265")
    assert driver[:len(DRIVER)] == DRIVER
    assert driver[len(DRIVER):] == ["--ros-args", "-r",
                                    "/wrist_camera/color/image_raw/compressed:=/wrist_camera/color/image_raw/ffmpeg"]
    assert (packets, image) == ("/wrist_camera/color/image_raw/ffmpeg", "/wrist_camera/color/image_raw")
    assert decoder[:3] == [rt.REPUBLISH, "ffmpeg", "raw"]
    assert "in/ffmpeg:=/wrist_camera/color/image_raw/ffmpeg" in decoder
    assert "out:=/wrist_camera/color/image_raw" in decoder
    assert "__node:=wrist_camera_color_decoder" in decoder
    assert "in.ffmpeg.decoders.libx265:=hevc" in decoder  # keyed by packet label (needs the driver label fix)


def test_h264_decoder():
    assert "in.ffmpeg.decoders.libx264:=h264" in rt.encoded_commands(DRIVER, "h264")[1]


def test_unknown_codec_exits_nonzero():
    r = subprocess.run([sys.executable, os.path.join(HERE, "..", "ready_timeout.py"), "true"],
                       env={**os.environ, "RAMMP_OAK_COLOR_CODEC": "vp9"}, capture_output=True, text=True)
    assert r.returncode != 0 and "vp9" in r.stderr


def test_raw_mode_passes_exit_code_through():
    r = subprocess.run([sys.executable, os.path.join(HERE, "..", "ready_timeout.py"), sys.executable, "-c",
                        "print('Driver ready!'); raise SystemExit(3)"],
                       env={k: v for k, v in os.environ.items() if k != "RAMMP_OAK_COLOR_CODEC"},
                       capture_output=True, text=True)
    assert r.returncode == 3 and "Driver ready!" in r.stdout
