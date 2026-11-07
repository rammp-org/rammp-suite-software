#!/usr/bin/env python3
"""Solve the three camera calibrations from one capture dataset.

Input is a dataset directory written by capture_calibration.py: one
station_NNN/ per arm pose, each holding wrist.png / scene.png, their
camera_info JSON, and transforms.json with the FK pose at capture time.
The checkerboard must have stayed fixed for the whole session.

Outputs, in order:
  1. Intrinsics verification per camera — per-view solvePnP reprojection RMS
     against the factory K/D from camera_info. Low RMS = keep factory.
  2. Wrist hand-eye (eye-in-hand): cv2.calibrateHandEye over
     (T_base<-ee, T_wristcam<-board) pairs -> T_ee<-wristcam.
  3. Scene extrinsic (eye-to-hand): the board's base-frame pose chained
     through (2), combined with the scene camera's own view of the board
     -> T_base<-scenecam.

All 4x4 matrices follow the T_parent<-child convention: X_parent = T @ X_child.
solvePnP yields T_cam<-board; FK yields T_base<-ee. Results are reported in
the camera OPTICAL frame (z forward, x right, y down) and, when the capture
recorded the driver's internal optical->root transform, also composed to the
driver's root frame — the value the mounts config wants.

Usage:
  uv run solve_calibration.py DATASET_DIR --cols 8 --rows 5 --square-size 0.030
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np
import yaml

# ---------------------------------------------------------------- transforms


def tmat(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t).reshape(3)
    return T


def inv_t(T: np.ndarray) -> np.ndarray:
    R, t = T[:3, :3], T[:3, 3]
    return tmat(R.T, -R.T @ t)


def quat_to_mat(q_xyzw) -> np.ndarray:
    x, y, z, w = np.asarray(q_xyzw, dtype=float) / np.linalg.norm(q_xyzw)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def mat_to_quat(R: np.ndarray) -> np.ndarray:
    """Rotation matrix -> quaternion [x, y, z, w] (Shepperd's method)."""
    m = R
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w, x = 0.25 * s, (m[2, 1] - m[1, 2]) / s
        y, z = (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w, x = (m[2, 1] - m[1, 2]) / s, 0.25 * s
        y, z = (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w, x = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s
        y, z = 0.25 * s, (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w, x = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s
        y, z = (m[1, 2] + m[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def xyzquat_to_mat(d: dict) -> np.ndarray:
    return tmat(quat_to_mat(d["quat_xyzw"]), d["xyz"])


def rot_angle_deg(R: np.ndarray) -> float:
    c = max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0))
    return math.degrees(math.acos(c))


def average_transforms(Ts: list[np.ndarray]) -> np.ndarray:
    """Mean translation + quaternion average (max-eigenvector of sum q q^T)."""
    t = np.mean([T[:3, 3] for T in Ts], axis=0)
    quats = [mat_to_quat(T[:3, :3]) for T in Ts]
    ref = quats[0]
    M = np.zeros((4, 4))
    for q in quats:
        if np.dot(q, ref) < 0:
            q = -q
        M += np.outer(q, q)
    q_mean = np.linalg.eigh(M)[1][:, -1]
    return tmat(quat_to_mat(q_mean), t)


def spread(Ts: list[np.ndarray], T_mean: np.ndarray) -> tuple[float, float]:
    """(translation std over stations in mm, max rotation deviation in deg)."""
    t_std = float(np.std([T[:3, 3] - T_mean[:3, 3] for T in Ts], axis=0).max()) * 1e3
    r_max = max(rot_angle_deg(T[:3, :3].T @ T_mean[:3, :3]) for T in Ts)
    return t_std, r_max


# ---------------------------------------------------------------- board + PnP


def board_object_points(cols: int, rows: int, square: float) -> np.ndarray:
    """Inner-corner grid in the board frame (z=0 plane), row-major like OpenCV."""
    g = np.zeros((rows * cols, 3), np.float64)
    g[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square
    return g


def detect_corners(gray: np.ndarray, cols: int, rows: int) -> np.ndarray | None:
    """(N,2) subpixel corners in OpenCV's canonical order, or None."""
    ok, corners = cv2.findChessboardCornersSB(
        gray, (cols, rows), flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
    )
    if not ok:
        ok, corners = cv2.findChessboardCorners(gray, (cols, rows))
        if not ok:
            return None
        corners = cv2.cornerSubPix(
            gray,
            corners,
            (11, 11),
            (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-4),
        )
    return corners.reshape(-1, 2).astype(np.float64)


def charuco_board(
    cols: int, rows: int, square: float, marker: float, dict_name: str
) -> "cv2.aruco.CharucoBoard":
    """cols/rows are SQUARES here (ChArUco convention), not inner corners."""
    d = cv2.aruco.getPredefinedDictionary(
        getattr(cv2.aruco, f"DICT_{dict_name.upper()}")
    )
    return cv2.aruco.CharucoBoard((cols, rows), square, marker, d)


def make_detector(args):
    """-> detect(gray) returning (img_pts Nx2, obj_pts Nx3) or None.

    Checkerboard: all-or-nothing, full grid. ChArUco: any >= 8 identified
    corners count — partial views are the point of that board."""
    if args.board == "checker":
        objpts = board_object_points(args.cols, args.rows, args.square_size)

        def detect(gray):
            corners = detect_corners(gray, args.cols, args.rows)
            return None if corners is None else (corners, objpts)

    else:
        board = charuco_board(
            args.cols, args.rows, args.square_size, args.marker_size, args.dict
        )
        detector = cv2.aruco.CharucoDetector(board)
        all_obj = board.getChessboardCorners().astype(np.float64)

        def detect(gray):
            corners, ids, _, _ = detector.detectBoard(gray)
            if corners is None or ids is None or len(ids) < 8:
                return None
            return (
                corners.reshape(-1, 2).astype(np.float64),
                all_obj[ids.reshape(-1)],
            )

    return detect


def pnp_board_pose(
    corners: np.ndarray, objpts: np.ndarray, K: np.ndarray, D: np.ndarray
) -> tuple[np.ndarray, float]:
    """solvePnP on the planar board -> (T_cam<-board, reprojection RMS px)."""
    ok, rvec, tvec = cv2.solvePnP(objpts, corners, K, D, flags=cv2.SOLVEPNP_IPPE)
    if not ok:
        raise RuntimeError("solvePnP failed")
    rvec, tvec = cv2.solvePnPRefineLM(objpts, corners, K, D, rvec, tvec)
    proj, _ = cv2.projectPoints(objpts, rvec, tvec, K, D)
    rms = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - corners) ** 2, axis=1))))
    R, _ = cv2.Rodrigues(rvec)
    return tmat(R, tvec), rms


# ---------------------------------------------------------------- the solves

HAND_EYE_METHODS = {
    "park": cv2.CALIB_HAND_EYE_PARK,
    "daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def hand_eye(
    T_base_ee: list[np.ndarray], T_cam_board: list[np.ndarray]
) -> dict[str, np.ndarray]:
    """cv2.calibrateHandEye per method -> {method: T_ee<-cam}."""
    R_g2b = [T[:3, :3] for T in T_base_ee]
    t_g2b = [T[:3, 3] for T in T_base_ee]
    R_t2c = [T[:3, :3] for T in T_cam_board]
    t_t2c = [T[:3, 3] for T in T_cam_board]
    out = {}
    for name, flag in HAND_EYE_METHODS.items():
        R, t = cv2.calibrateHandEye(R_g2b, t_g2b, R_t2c, t_t2c, method=flag)
        out[name] = tmat(R, t)
    return out


def board_in_base(
    T_base_ee: list[np.ndarray],
    T_ee_cam: np.ndarray,
    T_cam_board: list[np.ndarray],
) -> tuple[np.ndarray, float, float]:
    """Chain every station to the (static) board -> (mean, t_std_mm, rot_max_deg).

    The spread IS the hand-eye validation: a perfect calibration chains every
    station to the identical board pose."""
    Ts = [Tb @ T_ee_cam @ Tc for Tb, Tc in zip(T_base_ee, T_cam_board)]
    T_mean = average_transforms(Ts)
    t_std, r_max = spread(Ts, T_mean)
    return T_mean, t_std, r_max


def scene_from_board(
    T_base_board: np.ndarray, T_scene_board: list[np.ndarray]
) -> tuple[np.ndarray, float, float]:
    """T_base<-scene from each scene view of the static board, averaged."""
    Ts = [T_base_board @ inv_t(T) for T in T_scene_board]
    T_mean = average_transforms(Ts)
    t_std, r_max = spread(Ts, T_mean)
    return T_mean, t_std, r_max


def rotation_diversity_deg(T_base_ee: list[np.ndarray]) -> float:
    """Max pairwise ee rotation difference — the hand-eye conditioning check."""
    worst = 0.0
    for i in range(len(T_base_ee)):
        for j in range(i + 1, len(T_base_ee)):
            worst = max(
                worst, rot_angle_deg(T_base_ee[i][:3, :3].T @ T_base_ee[j][:3, :3])
            )
    return worst


# ---------------------------------------------------------------- dataset I/O


@dataclass
class Station:
    name: str
    T_base_ee: np.ndarray
    images: dict[str, Path] = field(default_factory=dict)  # cam -> png
    infos: dict[str, dict] = field(default_factory=dict)  # cam -> camera_info
    # cam -> T_root<-optical from the driver's own TF, when capture saw it
    root_from_optical: dict[str, np.ndarray] = field(default_factory=dict)


def load_dataset(root: Path) -> list[Station]:
    stations = []
    for d in sorted(root.glob("station_*")):
        tf = json.loads((d / "transforms.json").read_text())
        st = Station(name=d.name, T_base_ee=xyzquat_to_mat(tf["base_to_ee"]))
        for cam in ("wrist", "scene"):
            img = d / f"{cam}.png"
            info = d / f"{cam}_camera_info.json"
            if img.exists() and info.exists():
                st.images[cam] = img
                st.infos[cam] = json.loads(info.read_text())
            rfo = tf.get(f"{cam}_root_from_optical")
            if rfo:
                st.root_from_optical[cam] = xyzquat_to_mat(rfo)
        stations.append(st)
    if not stations:
        sys.exit(f"no station_*/ directories under {root}")
    return stations


def intrinsics(info: dict) -> tuple[np.ndarray, np.ndarray]:
    K = np.asarray(info["K"], dtype=float).reshape(3, 3)
    D = np.asarray(info.get("D") or [], dtype=float)
    return K, D


# ---------------------------------------------------------------- reporting


def fmt_transform(T: np.ndarray) -> str:
    x, y, z = T[:3, 3]
    q = mat_to_quat(T[:3, :3])
    return (
        f"xyz [{x:+.4f}, {y:+.4f}, {z:+.4f}]  "
        f"quat_xyzw [{q[0]:+.5f}, {q[1]:+.5f}, {q[2]:+.5f}, {q[3]:+.5f}]"
    )


def mount_entry(name: str, parent: str, child: str, T: np.ndarray) -> dict:
    q = mat_to_quat(T[:3, :3])
    return {
        name: {
            "parent": parent,
            "child": child,
            "xyz": [round(float(v), 6) for v in T[:3, 3]],
            "quat_xyzw": [round(float(v), 7) for v in q],
        }
    }


def write_mounts_yaml(path: Path, entries: dict) -> None:
    doc = {
        "mounts": {
            "ros__parameters": {"transforms": list(entries), **entries},
        }
    }
    path.write_text(
        "# Generated by rammp-calibration solve_calibration.py — review, then\n"
        "# land in the embodiment's configs/ in rammp-deployments.\n"
        + yaml.safe_dump(doc, sort_keys=False)
    )


# ---------------------------------------------------------------- main


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("dataset", type=Path)
    ap.add_argument("--board", choices=("checker", "charuco"), default="checker")
    ap.add_argument(
        "--cols", type=int, required=True,
        help="checker: inner corners per row; charuco: squares per row",
    )
    ap.add_argument(
        "--rows", type=int, required=True,
        help="checker: inner corners per column; charuco: squares per column",
    )
    ap.add_argument(
        "--square-size", type=float, required=True,
        help="meters, as MEASURED on the printed board",
    )
    ap.add_argument("--marker-size", type=float, help="charuco: marker side, meters")
    ap.add_argument("--dict", default="4X4_100", help="charuco: aruco dictionary")
    ap.add_argument("--ee-frame", default="end_effector_link")
    ap.add_argument("--base-frame", default="base_link")
    ap.add_argument("--out", type=Path, help="directory for mounts YAML output")
    args = ap.parse_args()

    if args.board == "charuco" and not args.marker_size:
        ap.error("--board charuco requires --marker-size")
    if args.board == "checker" and args.cols == args.rows:
        print(
            f"WARNING: a {args.cols}x{args.rows} grid is 180-degree ambiguous; "
            "corner order may flip between views and poison every solve."
        )

    detect = make_detector(args)
    stations = load_dataset(args.dataset)
    print(f"{len(stations)} stations in {args.dataset}")

    # --- detection + PnP, and intrinsics verification, per camera ----------
    obs: dict[str, dict[str, np.ndarray]] = {"wrist": {}, "scene": {}}
    for cam in ("wrist", "scene"):
        rms_all = []
        for st in stations:
            if cam not in st.images:
                continue
            gray = cv2.imread(str(st.images[cam]), cv2.IMREAD_GRAYSCALE)
            det = detect(gray)
            if det is None:
                print(f"  {st.name}/{cam}: board not found — skipped")
                continue
            corners, view_objpts = det
            K, D = intrinsics(st.infos[cam])
            T, rms = pnp_board_pose(corners, view_objpts, K, D)
            obs[cam][st.name] = T
            rms_all.append(rms)
        if rms_all:
            print(
                f"[intrinsics] {cam}: {len(rms_all)} views, reprojection RMS "
                f"mean {np.mean(rms_all):.3f} px, worst {max(rms_all):.3f} px "
                f"({'factory K ok' if max(rms_all) < 0.8 else 'HIGH — recheck K/resolution/board size'})"
            )

    # --- wrist hand-eye ----------------------------------------------------
    wrist_sts = [st for st in stations if st.name in obs["wrist"]]
    if len(wrist_sts) < 3:
        sys.exit(f"only {len(wrist_sts)} usable wrist views — need >= 3 (want 15+)")
    if len(wrist_sts) < 10:
        print(f"WARNING: only {len(wrist_sts)} wrist views; 15+ recommended")
    T_base_ee = [st.T_base_ee for st in wrist_sts]
    T_wrist_board = [obs["wrist"][st.name] for st in wrist_sts]

    div = rotation_diversity_deg(T_base_ee)
    if div < 20:
        print(
            f"WARNING: max pairwise ee rotation is {div:.1f} deg — hand-eye is "
            "poorly conditioned below ~20-30 deg; add rotated stations."
        )

    solutions = hand_eye(T_base_ee, T_wrist_board)
    names = list(solutions)
    T_ee_cam = solutions["park"]
    d_t = np.linalg.norm(solutions[names[0]][:3, 3] - solutions[names[1]][:3, 3]) * 1e3
    d_r = rot_angle_deg(solutions[names[0]][:3, :3].T @ solutions[names[1]][:3, :3])
    print(f"[hand-eye] {names[0]} vs {names[1]}: {d_t:.2f} mm / {d_r:.3f} deg apart")

    T_base_board, t_std, r_max = board_in_base(T_base_ee, T_ee_cam, T_wrist_board)
    print(
        f"[hand-eye] board-in-base consistency over {len(wrist_sts)} stations: "
        f"t std {t_std:.2f} mm, rot max dev {r_max:.3f} deg"
    )
    print(f"[hand-eye] {args.ee_frame} -> wrist optical: {fmt_transform(T_ee_cam)}")

    # --- scene extrinsic ---------------------------------------------------
    T_base_scene = None
    scene_views = [obs["scene"][n] for n in sorted(obs["scene"])]
    if scene_views:
        T_base_scene, s_std, s_rmax = scene_from_board(T_base_board, scene_views)
        print(
            f"[scene] {len(scene_views)} views: t std {s_std:.2f} mm, "
            f"rot max dev {s_rmax:.3f} deg"
        )
        print(f"[scene] {args.base_frame} -> scene optical: {fmt_transform(T_base_scene)}")
    else:
        print("[scene] board never detected in the scene camera — no scene solve")

    # --- compose to the drivers' root frames, write mounts YAML ------------
    if args.out:
        args.out.mkdir(parents=True, exist_ok=True)
        entries = {}
        for cam, T_parent_optical, parent in (
            ("wrist", T_ee_cam, args.ee_frame),
            ("scene", T_base_scene, args.base_frame),
        ):
            if T_parent_optical is None:
                continue
            rfo = next(
                (st.root_from_optical[cam] for st in stations if cam in st.root_from_optical),
                None,
            )
            frame_id = next(
                (st.infos[cam]["frame_id"] for st in stations if cam in st.infos), ""
            )
            if rfo is not None:
                # T_parent<-root = T_parent<-optical @ T_optical<-root
                T = T_parent_optical @ inv_t(rfo)
                child = f"{cam}_camera_link"
            else:
                print(
                    f"NOTE: no {cam} optical->root transform in the capture; the "
                    "mounts entry targets the OPTICAL frame — parent the driver "
                    "tree accordingly or re-capture with the driver TF up."
                )
                T, child = T_parent_optical, frame_id or f"{cam}_optical_frame"
            entries.update(mount_entry(f"{cam}_camera", parent, child, T))
        out = args.out / "mounts.yaml"
        write_mounts_yaml(out, entries)
        print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
