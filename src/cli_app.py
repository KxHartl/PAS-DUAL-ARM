#!/usr/bin/env python3
"""
Interactive Pick-and-Place CLI Application (Updated for Multi-Fruit Precision)
=============================================================================

Wraps the voce_zadaca pipeline with visual feedback and user interaction.
Supports DBSCAN multi-fruit detection and precise matrix-based stitching.

Usage:
    python -X utf8 src/cli_app.py                              # Full mode (robot + camera)
    python -X utf8 src/cli_app.py --offline                    # Offline mode (no robot/camera)
"""
from __future__ import annotations

import argparse
import sys
import time
from collections import defaultdict
from dataclasses import asdict
from pathlib import Path
from typing import Any

# Ensure sibling modules are importable regardless of CWD
# We add both 'src' and 'voce_zadaca' to path
ROOT_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT_DIR / "voce_zadaca"))
sys.path.insert(0, str(ROOT_DIR / "src"))

import cv2
import numpy as np

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt

from ultralytics import YOLO
from PIL import Image, ImageDraw

import open3d as o3d

from config import cfg
from io_utils import (
    ensure_dir,
    save_json,
    save_rgb_image,
    save_point_cloud,
    make_point_cloud,
    load_rgb_image,
)
from capture_scene import (
    CaptureSceneResult,
    CaptureViewResult,
    _connect_realsense,
    _connect_rtde_receive,
    _read_actual_tcp_pose,
    _capture_rgb_depth_and_intrinsics,
    _rgbd_to_organized_point_cloud,
    _as_pose6,
    pose6_to_matrix,
    get_pairs_and_transforms_from_capture_result,
    load_capture_scene_result,
    _pose_error,
    _build_movej_command,
    _wrap_program,
    _send_program,
)
from reconstruct_scene import reconstruct_scene_from_pairs
from estimate_pick_pose import estimate_pick_pose_from_reconstruction
from trajectory_planner import plan_pick_place_trajectory
from ur_executor import execute_pick_place_program


# ═══════════════════════════════════════════════════════════════════
#  ANSI Terminal Colors & Formatting Helpers
# ═══════════════════════════════════════════════════════════════════

class C:
    """ANSI escape codes for coloured terminal output."""
    RESET   = "\033[0m"
    BOLD    = "\033[1m"
    DIM     = "\033[2m"
    RED     = "\033[91m"
    GREEN   = "\033[92m"
    YELLOW  = "\033[93m"
    BLUE    = "\033[94m"
    MAGENTA = "\033[95m"
    CYAN    = "\033[96m"
    WHITE   = "\033[97m"
    BG_GREEN = "\033[42m"
    BG_BLUE  = "\033[44m"
    BG_RED   = "\033[41m"


def banner():
    print(f"""
{C.CYAN}{C.BOLD}╔══════════════════════════════════════════════════════════════╗
║      🍎  FRUIT PICK-AND-PLACE  —  Interactive CLI  🍊       ║
║                  UR5e  +  RealSense  +  YOLOv8               ║
╚══════════════════════════════════════════════════════════════╝{C.RESET}
""")


def step_header(step_num: int, total: int, title: str):
    print(f"\n{C.BG_BLUE}{C.WHITE}{C.BOLD}  KORAK {step_num}/{total}  {C.RESET}  "
          f"{C.BOLD}{title}{C.RESET}")
    print(f"  {C.DIM}{'─' * 58}{C.RESET}")


def info(msg: str):
    print(f"  {C.CYAN}ℹ{C.RESET}  {msg}")


def success(msg: str):
    print(f"  {C.GREEN}✔{C.RESET}  {msg}")


def warn(msg: str):
    print(f"  {C.YELLOW}⚠{C.RESET}  {msg}")


def error(msg: str):
    print(f"  {C.RED}✖{C.RESET}  {msg}")


def _timestamp() -> str:
    return time.strftime("%Y%m%d_%H%M%S")


# ═══════════════════════════════════════════════════════════════════
#  Colour palette for fruit classes
# ═══════════════════════════════════════════════════════════════════

_PALETTE_RGBA = [
    (230,  25,  75, 110),   # Red
    ( 60, 180,  75, 110),   # Green
    (  0, 130, 200, 110),   # Blue
    (255, 225,  25, 110),   # Yellow
    (245, 130,  48, 110),   # Orange
    (145,  30, 180, 110),   # Purple
    ( 70, 240, 240, 110),   # Cyan
    (240,  50, 230, 110),   # Magenta
]

# ═══════════════════════════════════════════════════════════════════
#  STEP 1 — Interactive Capture with Live Camera Display
# ═══════════════════════════════════════════════════════════════════

def _wait_while_updating_feed(
    rtde_receiver: Any,
    target_pose: np.ndarray,
    pipeline: Any,
    align: Any,
    win_name: str,
    view_idx: int,
    required_views: int,
    timeout_s: float = 20.0,
    pos_tol_m: float = 0.003,
    rot_tol_rad: float = 0.05,
    stable_cycles: int = 5,
    poll_dt_s: float = 0.05,
) -> np.ndarray:
    """Čeka da robot stigne u pozu dok istovremeno osvježava live sliku s kamere."""
    t0 = time.time()
    stable_count = 0
    last_pose = target_pose.copy()

    while time.time() - t0 < timeout_s:
        # Osvježi sliku s kamere
        try:
            frames = pipeline.wait_for_frames()
            aligned = align.process(frames)
            color_frame = aligned.get_color_frame()
            if color_frame:
                color_bgr = np.asanyarray(color_frame.get_data())
                overlay = color_bgr.copy()
                cv2.putText(
                    overlay,
                    f"Pomicem robota u View {view_idx + 1}/{required_views}...",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, (0, 165, 255), 2,
                )
                cv2.imshow(win_name, overlay)
        except Exception:
            pass
        cv2.waitKey(1)

        if rtde_receiver is not None:
            last_pose = _read_actual_tcp_pose(rtde_receiver)
            p_err, r_err = _pose_error(last_pose, target_pose)

            if p_err <= pos_tol_m and r_err <= rot_tol_rad:
                stable_count += 1
                if stable_count >= stable_cycles:
                    return last_pose
            else:
                stable_count = 0
        else:
            if time.time() - t0 > 1.5:
                return target_pose.copy()

        time.sleep(poll_dt_s)

    if rtde_receiver is not None:
        raise TimeoutError(f"Robot nije stigao u pozu unutar {timeout_s}s.")
    return target_pose.copy()


def _move_robot_to_pose_and_wait_with_feed(
    rtde_receiver: Any,
    target_pose: Any,
    cfg: Any,
    program_name: str,
    pipeline: Any,
    align: Any,
    win_name: str,
    view_idx: int,
    required_views: int,
    wait_timeout_s: float = 20.0,
    settle_time_s: float = 0.3,
) -> np.ndarray:
    """Šalje program kretanja i čeka uz prikaz live feeda."""
    target_pose = _as_pose6(target_pose, "target_pose")
    if rtde_receiver is not None:
        movej_cmd = _build_movej_command(target_pose, cfg)
        program = _wrap_program([movej_cmd], program_name=program_name)
        _send_program(program, cfg)

    # Čekamo da stigne u blizinu
    _wait_while_updating_feed(
        rtde_receiver=rtde_receiver,
        target_pose=target_pose,
        pipeline=pipeline,
        align=align,
        win_name=win_name,
        view_idx=view_idx,
        required_views=required_views,
        timeout_s=wait_timeout_s,
    )

    # Nastavi osvježavati feed tijekom smirivanja robota
    t_start = time.time()
    while time.time() - t_start < settle_time_s:
        try:
            frames = pipeline.wait_for_frames()
            aligned = align.process(frames)
            color_frame = aligned.get_color_frame()
            if color_frame:
                color_bgr = np.asanyarray(color_frame.get_data())
                overlay = color_bgr.copy()
                cv2.putText(
                    overlay,
                    f"Smirivanje robota...",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, (46, 204, 113), 2,
                )
                cv2.imshow(win_name, overlay)
        except Exception:
            pass
        cv2.waitKey(1)
        time.sleep(0.02)

    # KLJUČNO: Uzimamo pozu točno nakon smirivanja
    if rtde_receiver is not None:
        return _read_actual_tcp_pose(rtde_receiver)
    return target_pose.copy()


def interactive_capture(
    cfg: Any,
    run_dir: Path,
    pause_seconds: float = 2.0,
) -> CaptureSceneResult:
    capture_poses = list(cfg.capture.capture_poses)
    required_views = int(cfg.capture.required_views)
    output_dir = ensure_dir(run_dir / "captures")

    info(f"Broj capture poza: {required_views}")
    info(f"Output direktorij: {output_dir}")

    rtde_receiver = None
    if not cfg.runtime.offline_mode:
        info("Spajam se na robota...")
        rtde_receiver = _connect_rtde_receive(cfg)
        success(f"Robot spojen ({cfg.robot.robot_host})")
    else:
        warn("OFFLINE mod — robot nije spojen")

    info("Spajam se na RealSense kameru...")
    rs, pipeline, align, depth_scale = _connect_realsense(cfg)
    success("Kamera spojena i zagrijana")

    view_results: list[CaptureViewResult] = []
    pairs: list[list[str]] = []
    base_tcp_transform_paths: list[str] = []

    win_name = "Camera Live Feed"

    try:
        for view_idx in range(required_views):
            target_pose = _as_pose6(capture_poses[view_idx], f"capture_pose[{view_idx}]")
            view_dir = ensure_dir(output_dir / f"view_{view_idx:02d}")

            print(f"\n  {C.MAGENTA}📷 View {view_idx + 1}/{required_views}{C.RESET}")

            # Move robot with live feed
            actual_pose = _move_robot_to_pose_and_wait_with_feed(
                rtde_receiver=rtde_receiver,
                target_pose=target_pose,
                cfg=cfg,
                program_name=f"capture_{view_idx:02d}",
                pipeline=pipeline,
                align=align,
                win_name=win_name,
                view_idx=view_idx,
                required_views=required_views,
                wait_timeout_s=20.0,
                settle_time_s=0.5,
            )
            success("Robot stigao u poziciju")

            # Capture final frame
            info("Snimam RGB-D frame...")
            color_rgb, depth_m, intrinsics = _capture_rgb_depth_and_intrinsics(
                rs=rs, pipeline=pipeline, align=align, depth_scale=depth_scale,
            )
            pcd = _rgbd_to_organized_point_cloud(color_rgb, depth_m, intrinsics)

            # Persist to disk
            rgb_path = save_rgb_image(view_dir / "rgb.png", color_rgb)
            pcd_path = save_point_cloud(view_dir / "cloud.pcd", pcd)
            np.save(view_dir / "depth_m.npy", depth_m.astype(np.float32))

            t_base_tcp = pose6_to_matrix(actual_pose)
            t_base_tcp_path = view_dir / "T_base_tcp.npy"
            np.save(t_base_tcp_path, t_base_tcp)
            np.save(view_dir / "tcp_pose_base.npy", actual_pose.astype(np.float64))
            save_json(view_dir / "intrinsics.json", intrinsics)

            view_result = CaptureViewResult(
                view_index=view_idx, view_dir=str(view_dir),
                rgb_path=str(rgb_path), pcd_path=str(pcd_path),
                depth_npy_path=str(view_dir / "depth_m.npy"),
                tcp_pose_base=actual_pose.astype(float).tolist(),
                t_base_tcp_path=str(t_base_tcp_path),
            )
            view_results.append(view_result)
            pairs.append([str(rgb_path), str(pcd_path)])
            base_tcp_transform_paths.append(str(t_base_tcp_path))

            success(f"View {view_idx + 1} snimljen")

        # Return home
        if rtde_receiver is not None:
            info("Vraćam robota u HOME poziciju...")
            home_pose = _as_pose6(cfg.pick_place.home_pose, "home_pose")
            _move_robot_to_pose_and_wait_with_feed(
                rtde_receiver=rtde_receiver, target_pose=home_pose, cfg=cfg,
                program_name="capture_return_home", pipeline=pipeline, align=align,
                win_name=win_name, view_idx=required_views - 1, required_views=required_views,
            )

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

    result = CaptureSceneResult(
        captures_dir=str(output_dir), required_views=required_views,
        views_captured=len(view_results), pairs=pairs,
        base_tcp_transform_paths=base_tcp_transform_paths,
        view_results=view_results,
    )
    save_json(output_dir / "capture_scene_result.json", asdict(result))
    return result


# ═══════════════════════════════════════════════════════════════════
#  STEP 2 — Discover all fruit classes via YOLO
# ═══════════════════════════════════════════════════════════════════

def discover_all_fruits(capture_result: CaptureSceneResult, cfg: Any) -> list[dict]:
    model = YOLO(str(cfg.paths.segmentation_model))
    all_detections = []

    for vr in capture_result.view_results:
        result = model(str(vr.rgb_path), conf=cfg.target.confidence_threshold, verbose=False)[0]
        if result.masks is None: continue

        class_names = result.names
        classes = result.boxes.cls.cpu().numpy().astype(int)
        scores = result.boxes.conf.cpu().numpy()

        for i in range(len(classes)):
            all_detections.append({
                "class_name": class_names[classes[i]],
                "confidence": float(scores[i]),
            })
    return all_detections


def fruit_selection_menu(all_detections: list[dict]) -> str:
    by_class = defaultdict(list)
    for d in all_detections:
        by_class[d["class_name"]].append(d)

    sorted_classes = sorted(by_class.items(), key=lambda x: len(x[1]), reverse=True)

    print(f"\n  {C.BOLD}{'#':<4}{'Voće':<18}{'Detekcije':<12}{'Avg Conf':<20}{C.RESET}")
    print(f"  {'-' * 54}")
    for idx, (name, dets) in enumerate(sorted_classes, 1):
        avg_conf = np.mean([d["confidence"] for d in dets])
        print(f"  {C.CYAN}{idx:<4}{C.RESET}{name:<18}{len(dets):<12}{avg_conf:<20.2f}")

    while True:
        try:
            choice = input(f"\n  {C.YELLOW}Odaberi voće (1–{len(sorted_classes)}): {C.RESET}").strip()
            return sorted_classes[int(choice) - 1][0]
        except Exception:
            warn("Nevažeći unos.")


# ═══════════════════════════════════════════════════════════════════
#  STEP 4 — Execution & Analysis
# ═══════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--offline", action="store_true")
    parser.add_argument("--existing-captures", type=str, default=None)
    parser.add_argument("--invert-cam-tcp", action="store_true")
    args = parser.parse_args()

    if args.offline: cfg.runtime.offline_mode = True

    banner()
    run_dir = ensure_dir(Path(cfg.paths.output_dir) / f"run_{_timestamp()}")

    # ── STEP 1 ──────────────────────────────────────────────────
    step_header(1, 5, "SNIMANJE SCENE")
    if args.existing_captures:
        capture_result = load_capture_scene_result(args.existing_captures)
    else:
        capture_result = interactive_capture(cfg, run_dir)

    # ── STEP 2 & 3 ──────────────────────────────────────────────
    step_header(2, 5, "DETEKCIJA I ODABIR")
    all_detections = discover_all_fruits(capture_result, cfg)
    if not all_detections:
        error("Nema voća!")
        return

    # Use configured target class instead of interactive selection
    selected_class = cfg.target.target_class
    detected_classes = [d["class_name"] for d in all_detections]
    if selected_class not in detected_classes:
        warn(f"Konfigurirana klasa '{selected_class}' nije detektirana. Koristim najčešće detektiranu klasu umjesto toga.")
        # fallback: choose most frequent detected class
        selected_class = max(set(detected_classes), key=lambda c: sum(1 for x in detected_classes if x == c))
        cfg.target.target_class = selected_class
    info(f"Odabrana klasa za obradu: {selected_class}")

    # ── STEP 4 ──────────────────────────────────────────────────
    step_header(4, 5, "3D REKONSTRUKCIJA (DBSCAN)")
    pairs, transforms = get_pairs_and_transforms_from_capture_result(capture_result)

    recon_res = reconstruct_scene_from_pairs(
        pairs=pairs, base_tcp_transforms=transforms, cfg=cfg,
        segment_output_dir=run_dir / "segment",
        reconstruct_output_dir=run_dir / "reconstruct",
        invert_cam_tcp=args.invert_cam_tcp
    )

    info(f"Pronađeno {len(recon_res.instances)} zasebnih {selected_class} objekata.")

    # Visualization removed for automated runs (no interactive display)

    # Estimiranje prve voćke
    pick_pose_results = estimate_pick_pose_from_reconstruction(recon_res, cfg, run_dir / "pick_poses")
    target_pick = pick_pose_results[0]

    # ── STEP 5 ──────────────────────────────────────────────────
    step_header(5, 5, "IZVRŠENJE")
    traj = plan_pick_place_trajectory(target_pick, cfg, run_dir / "traj")

    # Execute pick-and-place program immediately (no interactive confirmation)
    execute_pick_place_program(traj, cfg, send_to_robot=not cfg.runtime.offline_mode)
    success("Program završen.")

    print(f"\n{C.BG_GREEN}{C.WHITE}{C.BOLD} PIPELINE GOTOV {C.RESET}")
    print(f"Centroid: {target_pick.centroid_base_xyz}")

if __name__ == "__main__":
    main()
