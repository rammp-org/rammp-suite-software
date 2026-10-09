#!/usr/bin/env python3
"""Capture a calibration dataset on the cell: images + FK per arm station.

Runs against the live stack (arm driver + RSP + both cameras up — the `full`
profile) with the checkerboard clamped on the bench, visible to the scene
camera and reachable by the wrist camera. Writes the dataset
solve_calibration.py eats:

  OUT/station_NNN/{wrist.png, scene.png, wrist_camera_info.json,
                   scene_camera_info.json, transforms.json}

Two ways to drive the arm:
  --poses poses.yaml   go_to_joint_config through each station (needs planner)
  --manual             you position the arm (teleop/freedrive); Enter captures,
                       q finishes. Interactive stdin required.

Uses system Python + ROS (rclpy, tf2_ros) — not the uv env. No cv_bridge:
images are decoded by hand so the only extra dependency is cv2/numpy, which
ROS Humble ships.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

# ------------------------------------------------- pure helpers (unit-tested)


def image_to_bgr(msg) -> np.ndarray:
    """sensor_msgs/Image -> BGR ndarray, without cv_bridge."""
    h, w, step = msg.height, msg.width, msg.step
    buf = np.frombuffer(msg.data, np.uint8).reshape(h, step)
    if msg.encoding in ("bgr8", "rgb8"):
        img = buf[:, : w * 3].reshape(h, w, 3)
        return img[:, :, ::-1].copy() if msg.encoding == "rgb8" else img.copy()
    if msg.encoding == "mono8":
        return cv2.cvtColor(buf[:, :w].copy(), cv2.COLOR_GRAY2BGR)
    raise ValueError(f"unhandled image encoding {msg.encoding!r}")


def transform_to_dict(tf_msg) -> dict:
    t, r = tf_msg.transform.translation, tf_msg.transform.rotation
    return {"xyz": [t.x, t.y, t.z], "quat_xyzw": [r.x, r.y, r.z, r.w]}


def camera_info_to_dict(info) -> dict:
    return {
        "width": info.width,
        "height": info.height,
        "K": list(info.k),
        "D": list(info.d),
        "distortion_model": info.distortion_model,
        "frame_id": info.header.frame_id,
    }


# ------------------------------------------------------------------ ROS node


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", type=Path, required=True)
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--poses", type=Path, help="YAML: {stations: [[7 joints rad], ...]}")
    mode.add_argument("--manual", action="store_true")
    ap.add_argument("--wrist-topic", default="/wrist_camera/color/image_raw")
    ap.add_argument("--scene-topic", default="/scene_camera/color/image_raw")
    ap.add_argument("--base-frame", default="base_link")
    ap.add_argument("--ee-frame", default="end_effector_link")
    ap.add_argument("--wrist-root-frame", default="wrist_camera_link")
    ap.add_argument("--scene-root-frame", default="scene_camera_link")
    ap.add_argument("--settle", type=float, default=1.0, help="s after motion stops")
    ap.add_argument("--image-timeout", type=float, default=5.0)
    args = ap.parse_args()

    import rclpy
    import yaml
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image
    from tf2_ros import Buffer, TransformListener

    class Capture(Node):
        def __init__(self):
            super().__init__("calibration_capture")
            self.latest: dict[str, object] = {}
            for cam, topic in (("wrist", args.wrist_topic), ("scene", args.scene_topic)):
                info_topic = topic.rsplit("/", 1)[0] + "/camera_info"
                self.create_subscription(
                    Image, topic,
                    lambda m, c=cam: self.latest.__setitem__(c, m),
                    qos_profile_sensor_data,
                )
                self.create_subscription(
                    CameraInfo, info_topic,
                    lambda m, c=cam: self.latest.__setitem__(c + "_info", m),
                    qos_profile_sensor_data,
                )
            self.tf = Buffer()
            self._tf_listener = TransformListener(self.tf, self)

        def spin_for(self, seconds: float):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.05)

        def fresh_image(self, cam: str, after_ns: int):
            """Spin until an image stamped after `after_ns` arrives, or time out."""
            deadline = time.monotonic() + args.image_timeout
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                msg = self.latest.get(cam)
                if msg is not None:
                    stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
                    if stamp > after_ns:
                        return msg
            self.get_logger().warning(f"no fresh {cam} image within {args.image_timeout}s")
            return self.latest.get(cam)

        def lookup(self, target: str, source: str, required: bool):
            try:
                return self.tf.lookup_transform(
                    target, source, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=2.0),
                )
            except Exception as e:  # tf2 raises several, all equal here
                if required:
                    raise SystemExit(f"TF {target} <- {source} unavailable: {e}")
                self.get_logger().info(f"no TF {target} <- {source} ({e}) — skipped")
                return None

        def capture_station(self, idx: int) -> bool:
            d = args.out / f"station_{idx:03d}"
            d.mkdir(parents=True, exist_ok=True)
            now_ns = self.get_clock().now().nanoseconds
            tfs = {"base_to_ee": transform_to_dict(
                self.lookup(args.base_frame, args.ee_frame, required=True))}
            wrote_image = False
            for cam, root in (("wrist", args.wrist_root_frame),
                              ("scene", args.scene_root_frame)):
                msg = self.fresh_image(cam, now_ns)
                info = self.latest.get(cam + "_info")
                if msg is None or info is None:
                    self.get_logger().warning(f"{cam}: no image/camera_info — skipped")
                    continue
                cv2.imwrite(str(d / f"{cam}.png"), image_to_bgr(msg))
                (d / f"{cam}_camera_info.json").write_text(
                    json.dumps(camera_info_to_dict(info), indent=1))
                # The driver's own optical->root link, so the solve can emit the
                # mount in the root frame rather than the optical one.
                rfo = self.lookup(root, msg.header.frame_id, required=False)
                if rfo is not None:
                    tfs[f"{cam}_root_from_optical"] = transform_to_dict(rfo)
                wrote_image = True
            (d / "transforms.json").write_text(json.dumps(tfs, indent=1))
            self.get_logger().info(f"captured {d.name}")
            return wrote_image

    def goto(node: Capture, joints: list[float]) -> bool:
        from rammp_arm_interfaces.action import GoToJointConfig
        from rclpy.action import ActionClient

        client = ActionClient(node, GoToJointConfig, "go_to_joint_config")
        if not client.wait_for_server(timeout_sec=10.0):
            raise SystemExit("go_to_joint_config server not up — planner running?")
        goal = GoToJointConfig.Goal()
        goal.target_joints = [float(v) for v in joints]
        goal.sender_id = "calibration_capture"
        fut = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, fut)
        handle = fut.result()
        if handle is None or not handle.accepted:
            node.get_logger().error("goal rejected")
            return False
        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(node, res_fut)
        result = res_fut.result().result
        if result.error_code != 0:
            node.get_logger().error(
                f"go_to_joint_config failed: {result.error_code} {result.error_string}")
            return False
        return True

    rclpy.init()
    node = Capture()
    args.out.mkdir(parents=True, exist_ok=True)
    captured = 0
    try:
        node.get_logger().info("waiting 3 s for subscriptions and TF…")
        node.spin_for(3.0)
        if args.manual:
            idx = 0
            while True:
                ans = input(f"station {idx}: position the arm, Enter to capture, q to finish > ")
                if ans.strip().lower() == "q":
                    break
                node.spin_for(args.settle)
                captured += node.capture_station(idx)
                idx += 1
        else:
            stations = yaml.safe_load(args.poses.read_text())["stations"]
            node.get_logger().info(f"{len(stations)} stations from {args.poses}")
            for idx, joints in enumerate(stations):
                if not goto(node, joints):
                    node.get_logger().warning(f"station {idx}: move failed — skipped")
                    continue
                node.spin_for(args.settle)
                captured += node.capture_station(idx)
    finally:
        node.get_logger().info(f"{captured} stations captured -> {args.out}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if captured >= 3 else 1


if __name__ == "__main__":
    sys.exit(main())
