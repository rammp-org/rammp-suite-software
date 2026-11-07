"""Round-trip tests: synthesize a world with known ground truth, recover it.

Two levels:
  - pure math: corners synthesized with cv2.projectPoints, no images;
    asserts tight recovery of every transform including the board pose.
  - end to end: checkerboard images rendered with warpPerspective, dataset
    written to disk, solve_calibration.main() run on it; asserts the mounts
    YAML lands within tolerance of truth. (Corner-detection origin choice may
    redefine the board frame consistently across views — hand-eye and scene
    results are invariant to that, so only those are asserted here.)
"""

import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

import solve_calibration as sc

RNG = np.random.default_rng(7)

COLS, ROWS, SQUARE = 8, 5, 0.03
K = np.array([[600.0, 0, 320], [0, 600.0, 240], [0, 0, 1]])
D = np.zeros(5)
OBJ = sc.board_object_points(COLS, ROWS, SQUARE)


def rxyz(ax, ay, az):
    """Rotation from euler xyz, degrees."""
    r = cv2.Rodrigues(np.array([math.radians(ax), 0.0, 0.0]))[0]
    r = r @ cv2.Rodrigues(np.array([0.0, math.radians(ay), 0.0]))[0]
    return r @ cv2.Rodrigues(np.array([0.0, 0.0, math.radians(az)]))[0]


# Ground truth. The scene camera's view of the board is chosen directly (a
# well-framed view, like the wrist ones); its base pose follows from the board.
T_EE_CAM = sc.tmat(rxyz(10, -5, 90), [0.05, -0.03, 0.08])
T_BASE_BOARD = sc.tmat(rxyz(180, 0, 15), [0.55, -0.10, 0.02])
T_SCENE_BOARD = sc.tmat(
    rxyz(12, -8, 20), [-COLS / 2 * SQUARE, -ROWS / 2 * SQUARE, 0.85]
)
T_BASE_SCENE = T_BASE_BOARD @ sc.inv_t(T_SCENE_BOARD)


def wrist_cam_board_poses(n=12):
    """Diverse board-in-camera poses, all corners well inside a 640x480 frame."""
    poses = []
    for i in range(n):
        ax, ay = 22 * math.sin(2.1 * i + 0.5), 22 * math.cos(1.3 * i)
        az = (i / (n - 1) - 0.5) * 80
        tz = 0.40 + 0.18 * (i % 4) / 3
        # Keep the board roughly centered: its middle is at (cols, rows)/2*square
        # in the board frame, so offset the translation by minus that.
        t = [
            -COLS / 2 * SQUARE + 0.02 * math.sin(3 * i),
            -ROWS / 2 * SQUARE + 0.02 * math.cos(2 * i),
            tz,
        ]
        poses.append(sc.tmat(rxyz(ax, ay, az), t))
    return poses


def project(T_cam_board, noise=0.0):
    rvec = cv2.Rodrigues(T_cam_board[:3, :3])[0]
    pts, _ = cv2.projectPoints(OBJ, rvec, T_cam_board[:3, 3], K, D)
    pts = pts.reshape(-1, 2)
    if noise:
        pts = pts + RNG.normal(0, noise, pts.shape)
    return pts


def angle_between(Ta, Tb):
    return sc.rot_angle_deg(Ta[:3, :3].T @ Tb[:3, :3])


# ------------------------------------------------------------------- helpers


def test_quat_roundtrip():
    for _ in range(50):
        R = cv2.Rodrigues(RNG.normal(0, 1.5, 3))[0]
        R2 = sc.quat_to_mat(sc.mat_to_quat(R))
        assert np.allclose(R, R2, atol=1e-9)


def test_average_transforms_recovers_mean():
    T = sc.tmat(rxyz(30, -20, 45), [1, 2, 3])
    Ts = [T @ sc.tmat(cv2.Rodrigues(RNG.normal(0, 0.002, 3))[0], RNG.normal(0, 1e-4, 3))
          for _ in range(40)]
    T_mean = sc.average_transforms(Ts)
    assert np.linalg.norm(T_mean[:3, 3] - T[:3, 3]) < 1e-3
    assert angle_between(T_mean, T) < 0.1


# ----------------------------------------------------------------- pure math


def test_pnp_recovers_pose():
    T = wrist_cam_board_poses()[0]
    T_hat, rms = sc.pnp_board_pose(project(T, noise=0.15), OBJ, K, D)
    assert rms < 0.5
    assert np.linalg.norm(T_hat[:3, 3] - T[:3, 3]) < 1.5e-3
    assert angle_between(T_hat, T) < 0.3


def test_hand_eye_and_scene_roundtrip():
    cam_board = wrist_cam_board_poses()
    # FK consistent with the world: T_base_ee = T_base_board @ inv(T_cam_board) @ inv(T_ee_cam)
    base_ee = [T_BASE_BOARD @ sc.inv_t(T) @ sc.inv_t(T_EE_CAM) for T in cam_board]
    pnp = [sc.pnp_board_pose(project(T, noise=0.15), OBJ, K, D)[0] for T in cam_board]

    assert sc.rotation_diversity_deg(base_ee) > 30

    sols = sc.hand_eye(base_ee, pnp)
    for name, T_hat in sols.items():
        assert np.linalg.norm(T_hat[:3, 3] - T_EE_CAM[:3, 3]) < 1e-3, name
        assert angle_between(T_hat, T_EE_CAM) < 0.1, name

    T_bb, t_std, r_max = sc.board_in_base(base_ee, sols["park"], pnp)
    assert np.linalg.norm(T_bb[:3, 3] - T_BASE_BOARD[:3, 3]) < 1.5e-3
    assert angle_between(T_bb, T_BASE_BOARD) < 0.15
    assert t_std < 1.0 and r_max < 0.2

    # Scene: static camera, many noisy views of the same static board.
    scene_pnp = [
        sc.pnp_board_pose(project(T_SCENE_BOARD, noise=0.15), OBJ, K, D)[0]
        for _ in range(8)
    ]
    T_bs, s_std, s_rmax = sc.scene_from_board(T_bb, scene_pnp)
    assert np.linalg.norm(T_bs[:3, 3] - T_BASE_SCENE[:3, 3]) < 2e-3
    assert angle_between(T_bs, T_BASE_SCENE) < 0.2


# ---------------------------------------------------------------- end to end


def render_board(T_cam_board, size=(640, 480)):
    """Render the checkerboard seen from T_cam<-board with warpPerspective."""
    p, m = 60, 120  # px per square, white margin px
    w, h = (COLS + 1) * p + 2 * m, (ROWS + 1) * p + 2 * m
    board = np.full((h, w), 255, np.uint8)
    for j in range(ROWS + 1):
        for i in range(COLS + 1):
            if (i + j) % 2 == 0:
                board[m + j * p : m + (j + 1) * p, m + i * p : m + (i + 1) * p] = 0
    # Inner corner (0,0) sits at source px (m+p, m+p); scale p px per SQUARE m.
    S = np.array([[p / SQUARE, 0, m + p], [0, p / SQUARE, m + p], [0, 0, 1]])
    Rt = np.column_stack(
        [T_cam_board[:3, 0], T_cam_board[:3, 1], T_cam_board[:3, 3]]
    )
    H = (K @ Rt) @ np.linalg.inv(S)
    return cv2.warpPerspective(
        board, H, size, flags=cv2.INTER_AREA, borderValue=255
    )


def write_info(path, frame_id):
    path.write_text(
        json.dumps(
            {
                "width": 640,
                "height": 480,
                "K": K.reshape(-1).tolist(),
                "D": D.tolist(),
                "distortion_model": "plumb_bob",
                "frame_id": frame_id,
            }
        )
    )


def xyzquat(T):
    return {
        "xyz": T[:3, 3].tolist(),
        "quat_xyzw": sc.mat_to_quat(T[:3, :3]).tolist(),
    }


def test_end_to_end_dataset(tmp_path, monkeypatch, capsys):
    cam_board = wrist_cam_board_poses()
    for i, T in enumerate(cam_board):
        d = tmp_path / f"station_{i:03d}"
        d.mkdir()
        cv2.imwrite(str(d / "wrist.png"), render_board(T))
        cv2.imwrite(str(d / "scene.png"), render_board(T_SCENE_BOARD))
        write_info(d / "wrist_camera_info.json", "wrist_camera_color_optical_frame")
        write_info(d / "scene_camera_info.json", "scene_camera_color_optical_frame")
        T_base_ee = T_BASE_BOARD @ sc.inv_t(T) @ sc.inv_t(T_EE_CAM)
        (d / "transforms.json").write_text(
            json.dumps({"base_to_ee": xyzquat(T_base_ee)})
        )

    out = tmp_path / "out"
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "solve_calibration.py",
            str(tmp_path),
            "--cols", str(COLS),
            "--rows", str(ROWS),
            "--square-size", str(SQUARE),
            "--out", str(out),
        ],
    )
    assert sc.main() == 0

    doc = yaml.safe_load((out / "mounts.yaml").read_text())
    params = doc["mounts"]["ros__parameters"]
    assert set(params["transforms"]) == {"wrist_camera", "scene_camera"}

    # Detection may redefine the board frame (consistently), so assert only the
    # board-invariant results: the two camera extrinsics.
    wrist = params["wrist_camera"]
    T_w = sc.tmat(sc.quat_to_mat(wrist["quat_xyzw"]), wrist["xyz"])
    assert wrist["parent"] == "end_effector_link"
    assert np.linalg.norm(T_w[:3, 3] - T_EE_CAM[:3, 3]) < 3e-3
    assert angle_between(T_w, T_EE_CAM) < 0.4

    scene = params["scene_camera"]
    T_s = sc.tmat(sc.quat_to_mat(scene["quat_xyzw"]), scene["xyz"])
    assert scene["parent"] == "base_link"
    assert np.linalg.norm(T_s[:3, 3] - T_BASE_SCENE[:3, 3]) < 5e-3
    assert angle_between(T_s, T_BASE_SCENE) < 0.5
