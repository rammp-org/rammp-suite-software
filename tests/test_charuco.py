"""ChArUco detection + PnP round trip on rendered views, incl. partial views."""

import math
from types import SimpleNamespace

import cv2
import numpy as np

import solve_calibration as sc
from test_solve import K, D, rxyz, angle_between

SQ_COLS, SQ_ROWS, SQUARE, MARKER = 8, 5, 0.03, 0.022

BOARD = sc.charuco_board(SQ_COLS, SQ_ROWS, SQUARE, MARKER, "4X4_100")
ARGS = SimpleNamespace(
    board="charuco", cols=SQ_COLS, rows=SQ_ROWS,
    square_size=SQUARE, marker_size=MARKER, dict="4X4_100",
)


def flat_board_and_scale(px_per_square=80, margin=160):
    """Render the flat board and fit the meters->pixels map by detecting it."""
    w = SQ_COLS * px_per_square + 2 * margin
    h = SQ_ROWS * px_per_square + 2 * margin
    flat = BOARD.generateImage((w, h), marginSize=margin, borderBits=1)
    corners, ids, _, _ = cv2.aruco.CharucoDetector(BOARD).detectBoard(flat)
    assert corners is not None and len(ids) == (SQ_COLS - 1) * (SQ_ROWS - 1)
    obj = BOARD.getChessboardCorners()[ids.reshape(-1)][:, :2].astype(np.float64)
    A, _ = cv2.estimateAffinePartial2D(obj, corners.reshape(-1, 2))
    S = np.vstack([A, [0, 0, 1]])
    return flat, S


FLAT, S = flat_board_and_scale()


def render(T_cam_board, size=(640, 480)):
    Rt = np.column_stack([T_cam_board[:3, 0], T_cam_board[:3, 1], T_cam_board[:3, 3]])
    H = (K @ Rt) @ np.linalg.inv(S)
    return cv2.warpPerspective(FLAT, H, size, flags=cv2.INTER_AREA, borderValue=255)


def centered_pose(ax, ay, az, tz, dx=0.0, dy=0.0):
    t = [-SQ_COLS / 2 * SQUARE + dx, -SQ_ROWS / 2 * SQUARE + dy, tz]
    return sc.tmat(rxyz(ax, ay, az), t)


def test_charuco_full_view_roundtrip():
    detect = sc.make_detector(ARGS)
    for T in (
        centered_pose(10, -15, 25, 0.55),
        centered_pose(-20, 8, -40, 0.70),
    ):
        det = detect(render(T))
        assert det is not None
        imgpts, objpts = det
        assert len(imgpts) >= 20
        T_hat, rms = sc.pnp_board_pose(imgpts, objpts, K, D)
        assert rms < 0.7
        assert np.linalg.norm(T_hat[:3, 3] - T[:3, 3]) < 2e-3
        assert angle_between(T_hat, T) < 0.3


def test_charuco_partial_view_still_solves():
    """Half the board out of frame — the case plain checkerboard loses."""
    detect = sc.make_detector(ARGS)
    # Pushed far enough off-center that a large part of the board leaves the
    # frame entirely.
    T = centered_pose(5, -10, 15, 0.35, dx=0.16)
    img = render(T)
    # The full-grid checkerboard detection must fail on this view, proving the
    # partial-view advantage.
    checker = sc.detect_corners(img, SQ_COLS - 1, SQ_ROWS - 1)
    assert checker is None
    det = detect(img)
    assert det is not None
    imgpts, objpts = det
    assert 8 <= len(imgpts) < len(BOARD.getChessboardCorners())
    T_hat, rms = sc.pnp_board_pose(imgpts, objpts, K, D)
    assert rms < 0.7
    assert np.linalg.norm(T_hat[:3, 3] - T[:3, 3]) < 5e-3
    assert angle_between(T_hat, T) < 0.6


def test_charuco_hand_eye_roundtrip():
    detect = sc.make_detector(ARGS)
    T_ee_cam = sc.tmat(rxyz(10, -5, 90), [0.05, -0.03, 0.08])
    T_base_board = sc.tmat(rxyz(180, 0, 15), [0.55, -0.10, 0.02])
    cam_board, pnp = [], []
    for i in range(10):
        T = centered_pose(
            18 * math.sin(2.1 * i + 0.5),
            18 * math.cos(1.3 * i),
            (i / 9 - 0.5) * 70,
            0.45 + 0.15 * (i % 3) / 2,
        )
        det = detect(render(T))
        assert det is not None, f"view {i} undetected"
        imgpts, objpts = det
        cam_board.append(T)
        pnp.append(sc.pnp_board_pose(imgpts, objpts, K, D)[0])
    base_ee = [T_base_board @ sc.inv_t(T) @ sc.inv_t(T_ee_cam) for T in cam_board]
    sols = sc.hand_eye(base_ee, pnp)
    for name, T_hat in sols.items():
        assert np.linalg.norm(T_hat[:3, 3] - T_ee_cam[:3, 3]) < 2e-3, name
        # ChArUco corners (marker-interpolated) are a touch noisier than SB
        # checkerboard corners; 0.3 deg on 10 rendered views is still tight.
        assert angle_between(T_hat, T_ee_cam) < 0.3, name
