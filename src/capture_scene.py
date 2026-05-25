# capture_scene.py
from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any
import socket
import time

import numpy as np
from scipy.spatial.transform import Rotation as R

from io_utils import ensure_dir, make_point_cloud, save_json, save_point_cloud, save_rgb_image


@dataclass
class CaptureViewResult:
    view_index: int
    view_dir: str
    rgb_path: str
    pcd_path: str
    depth_npy_path: str
    tcp_pose_base: list[float]
    t_base_tcp_path: str


@dataclass
class CaptureSceneResult:
    captures_dir: str
    required_views: int
    views_captured: int
    pairs: list[list[str]]
    base_tcp_transform_paths: list[str]
    view_results: list[CaptureViewResult]


def _as_pose6(vec: Any, name: str) -> np.ndarray:
    arr = np.asarray(vec, dtype=np.float64).reshape(-1)
    if arr.shape != (6,):
        raise ValueError(f"{name} mora imati 6 elemenata [x, y, z, rx, ry, rz], dobiveno {arr.shape}.")
    return arr


def pose6_to_matrix(pose6: Any) -> np.ndarray:
    pose6 = _as_pose6(pose6, "pose6")
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R.from_rotvec(pose6[3:]).as_matrix()
    T[:3, 3] = pose6[:3]
    return T


def load_base_tcp_transforms_from_capture_result(capture_result: CaptureSceneResult) -> list[np.ndarray]:
    transforms = []
    for path_str in capture_result.base_tcp_transform_paths:
        T = np.load(path_str)
        if T.shape != (4, 4):
            raise ValueError(f"Transformacija nije 4x4: {path_str}")
        transforms.append(T.astype(np.float64))
    return transforms


def get_pairs_and_transforms_from_capture_result(
    capture_result: CaptureSceneResult,
) -> tuple[list[tuple[str, str]], list[np.ndarray]]:
    pairs = [(item[0], item[1]) for item in capture_result.pairs]
    transforms = load_base_tcp_transforms_from_capture_result(capture_result)
    return pairs, transforms


def _wrap_program(lines: list[str], program_name: str = "prog") -> str:
    body = [line.rstrip() for line in lines if str(line).strip()]
    program = [f"def {program_name}():"]
    program.extend([f"  {line}" for line in body])
    program.append("end")
    program.append(f"{program_name}()")
    program.append("")
    return "\n".join(program)


def _pose_to_urscript(pose: Any) -> str:
    pose = _as_pose6(pose, "pose")
    x, y, z, rx, ry, rz = pose.tolist()
    return f"p[{x:.6f}, {y:.6f}, {z:.6f}, {rx:.6f}, {ry:.6f}, {rz:.6f}]"


def _build_movej_command(pose: Any, cfg: Any) -> str:
    pose_str = _pose_to_urscript(pose)
    a = float(cfg.robot.movej_acc)
    v = float(cfg.robot.movej_vel)
    return f"movej(get_inverse_kin({pose_str}), a={a:.6f}, v={v:.6f})"


def _send_program(program: str, cfg: Any, timeout_s: float = 5.0) -> None:
    host = str(cfg.robot.robot_host)
    port = int(cfg.robot.robot_port_script)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(timeout_s)
        s.connect((host, port))
        s.sendall(program.encode("utf-8"))


def _connect_rtde_receive(cfg: Any):
    try:
        from rtde_receive import RTDEReceiveInterface
    except ImportError as e:
        raise ImportError(
            "Nije moguće importati rtde_receive. "
            "Instaliraj ur-rtde ili prilagodi _connect_rtde_receive() "
            "vašem načinu dohvaćanja TCP poze."
        ) from e

    return RTDEReceiveInterface(str(cfg.robot.robot_host))


def _read_actual_tcp_pose(rtde_receiver) -> np.ndarray:
    pose = np.asarray(rtde_receiver.getActualTCPPose(), dtype=np.float64).reshape(-1)
    if pose.shape != (6,):
        raise RuntimeError(f"Robot nije vratio validan TCP pose6. Dobiven shape: {pose.shape}")
    return pose


def _pose_error(current_pose: np.ndarray, target_pose: np.ndarray) -> tuple[float, float]:
    p_err = float(np.linalg.norm(current_pose[:3] - target_pose[:3]))
    R_cur = R.from_rotvec(current_pose[3:])
    R_tgt = R.from_rotvec(target_pose[3:])
    rot_err = float(np.linalg.norm((R_cur.inv() * R_tgt).as_rotvec()))
    return p_err, rot_err


def _wait_until_robot_reaches_pose(
    rtde_receiver,
    target_pose: Any,
    timeout_s: float = 20.0,
    pos_tol_m: float = 0.003,
    rot_tol_rad: float = 0.05,
    stable_cycles: int = 5,
    poll_dt_s: float = 0.05,
) -> np.ndarray:
    target_pose = _as_pose6(target_pose, "target_pose")

    t0 = time.time()
    stable_count = 0
    last_pose = None

    while time.time() - t0 < timeout_s:
        last_pose = _read_actual_tcp_pose(rtde_receiver)
        p_err, r_err = _pose_error(last_pose, target_pose)

        if p_err <= pos_tol_m and r_err <= rot_tol_rad:
            stable_count += 1
            if stable_count >= stable_cycles:
                return last_pose
        else:
            stable_count = 0

        time.sleep(poll_dt_s)

    raise TimeoutError(
        f"Robot nije dosegao zadanu pozu unutar {timeout_s:.1f}s. "
        f"Zadnja pogreška: pos={p_err:.4f} m, rot={r_err:.4f} rad"
    )


def _move_robot_to_pose_and_wait(
    rtde_receiver,
    target_pose: Any,
    cfg: Any,
    program_name: str,
    wait_timeout_s: float = 20.0,
    settle_time_s: float = 0.3,
) -> np.ndarray:
    target_pose = _as_pose6(target_pose, "target_pose")
    movej_cmd = _build_movej_command(target_pose, cfg)
    program = _wrap_program([movej_cmd], program_name=program_name)
    _send_program(program, cfg)
    actual_pose = _wait_until_robot_reaches_pose(
        rtde_receiver=rtde_receiver,
        target_pose=target_pose,
        timeout_s=wait_timeout_s,
    )
    time.sleep(float(settle_time_s))
    return actual_pose


def _connect_realsense(cfg: Any):
    try:
        import pyrealsense2 as rs
    except ImportError as e:
        raise ImportError(
            "Nije moguće importati pyrealsense2. "
            "Instaliraj Intel RealSense Python paket ili prilagodi _connect_realsense()."
        ) from e

    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_stream(
        rs.stream.color,
        int(cfg.camera.color_width),
        int(cfg.camera.color_height),
        rs.format.bgr8,
        int(cfg.camera.color_fps),
    )
    config.enable_stream(
        rs.stream.depth,
        int(cfg.camera.depth_width),
        int(cfg.camera.depth_height),
        rs.format.z16,
        int(cfg.camera.depth_fps),
    )

    profile = pipeline.start(config)
    align = rs.align(rs.stream.color)

    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = float(depth_sensor.get_depth_scale())

    for _ in range(int(cfg.camera.warmup_frames)):
        pipeline.wait_for_frames()

    return rs, pipeline, align, depth_scale


def _capture_rgb_depth_and_intrinsics(rs, pipeline, align, depth_scale: float):
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)

    color_frame = aligned_frames.get_color_frame()
    depth_frame = aligned_frames.get_depth_frame()

    if not color_frame or not depth_frame:
        raise RuntimeError("Nisam uspio dohvatiti aligned color/depth frame s RealSense kamere.")

    color_bgr = np.asanyarray(color_frame.get_data())
    color_rgb = color_bgr[..., ::-1].copy()

    depth_raw = np.asanyarray(depth_frame.get_data()).astype(np.float32)
    depth_m = depth_raw * depth_scale

    intr = color_frame.profile.as_video_stream_profile().intrinsics
    intrinsics = {
        "fx": float(intr.fx),
        "fy": float(intr.fy),
        "cx": float(intr.ppx),
        "cy": float(intr.ppy),
        "width": int(intr.width),
        "height": int(intr.height),
    }
    return color_rgb, depth_m, intrinsics


def _rgbd_to_organized_point_cloud(
    color_rgb: np.ndarray,
    depth_m: np.ndarray,
    intrinsics: dict[str, float | int],
):
    h, w = depth_m.shape
    if color_rgb.shape[:2] != (h, w):
        raise ValueError(
            f"Dimenzije color i depth se ne poklapaju: color={color_rgb.shape[:2]}, depth={(h, w)}"
        )

    fx = float(intrinsics["fx"])
    fy = float(intrinsics["fy"])
    cx = float(intrinsics["cx"])
    cy = float(intrinsics["cy"])

    uu, vv = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))

    z = depth_m.astype(np.float32)
    x = (uu - cx) * z / fx
    y = (vv - cy) * z / fy

    points = np.stack([x, y, z], axis=-1).reshape(-1, 3).astype(np.float64)

    invalid = ~np.isfinite(points).all(axis=1) | (points[:, 2] <= 0.0)
    points[invalid] = 0.0

    colors = (color_rgb.reshape(-1, 3).astype(np.float32) / 255.0).clip(0.0, 1.0)

    pcd = make_point_cloud(points=points, colors=colors)
    return pcd


def capture_scene(
    cfg: Any,
    output_dir: str | Path | None = None,
    move_robot: bool = True,
    settle_time_s: float = 0.3,
    wait_timeout_s: float = 20.0,
) -> CaptureSceneResult:
    capture_poses = list(getattr(cfg.capture, "capture_poses", []))
    required_views = int(getattr(cfg.capture, "required_views", len(capture_poses)))

    if len(capture_poses) < required_views:
        raise ValueError(
            f"cfg.capture.capture_poses ima {len(capture_poses)} poza, "
            f"a cfg.capture.required_views traži {required_views}."
        )

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    output_dir = (
        Path(output_dir)
        if output_dir is not None
        else Path(cfg.paths.captures_dir) / f"scene_{timestamp}"
    )
    ensure_dir(output_dir)

    rtde_receiver = None
    if move_robot:
        rtde_receiver = _connect_rtde_receive(cfg)

    rs, pipeline, align, depth_scale = _connect_realsense(cfg)

    view_results: list[CaptureViewResult] = []
    pairs: list[list[str]] = []
    base_tcp_transform_paths: list[str] = []

    try:
        for view_index in range(required_views):
            target_pose = _as_pose6(capture_poses[view_index], f"cfg.capture.capture_poses[{view_index}]")
            view_dir = ensure_dir(output_dir / f"view_{view_index:02d}")

            if move_robot:
                _move_robot_to_pose_and_wait(
                    rtde_receiver=rtde_receiver,
                    target_pose=target_pose,
                    cfg=cfg,
                    program_name=f"capture_movej_{view_index:02d}",
                    wait_timeout_s=wait_timeout_s,
                    settle_time_s=settle_time_s,
                )
            else:
                if rtde_receiver is None:
                    rtde_receiver = _connect_rtde_receive(cfg)
                time.sleep(float(settle_time_s))

            # Citamo pozu s robota u trenutku slikanja za najbolju preciznost stichanja
            actual_pose = _read_actual_tcp_pose(rtde_receiver)

            color_rgb, depth_m, intrinsics = _capture_rgb_depth_and_intrinsics(
                rs=rs,
                pipeline=pipeline,
                align=align,
                depth_scale=depth_scale,
            )
            pcd = _rgbd_to_organized_point_cloud(
                color_rgb=color_rgb,
                depth_m=depth_m,
                intrinsics=intrinsics,
            )

            rgb_path = save_rgb_image(view_dir / "rgb.png", color_rgb)
            pcd_path = save_point_cloud(view_dir / "cloud.pcd", pcd)

            depth_npy_path = view_dir / "depth_m.npy"
            np.save(depth_npy_path, depth_m.astype(np.float32))

            t_base_tcp = pose6_to_matrix(actual_pose)
            t_base_tcp_path = view_dir / "T_base_tcp.npy"
            np.save(t_base_tcp_path, t_base_tcp)

            np.save(view_dir / "tcp_pose_base.npy", actual_pose.astype(np.float64))
            save_json(view_dir / "intrinsics.json", intrinsics)

            view_result = CaptureViewResult(
                view_index=view_index,
                view_dir=str(view_dir),
                rgb_path=str(rgb_path),
                pcd_path=str(pcd_path),
                depth_npy_path=str(depth_npy_path),
                tcp_pose_base=actual_pose.astype(float).tolist(),
                t_base_tcp_path=str(t_base_tcp_path),
            )
            view_results.append(view_result)
            pairs.append([str(rgb_path), str(pcd_path)])
            base_tcp_transform_paths.append(str(t_base_tcp_path))

        if move_robot:
            home_pose = _as_pose6(cfg.pick_place.home_pose, "cfg.pick_place.home_pose")
            _move_robot_to_pose_and_wait(
                rtde_receiver=rtde_receiver,
                target_pose=home_pose,
                cfg=cfg,
                program_name="capture_return_home",
                wait_timeout_s=wait_timeout_s,
                settle_time_s=settle_time_s,
            )

    finally:
        pipeline.stop()

    result = CaptureSceneResult(
        captures_dir=str(output_dir),
        required_views=required_views,
        views_captured=len(view_results),
        pairs=pairs,
        base_tcp_transform_paths=base_tcp_transform_paths,
        view_results=view_results,
    )

    save_json(output_dir / "capture_scene_result.json", asdict(result))
    return result


def load_capture_scene_result(
    captures_dir: str | Path,
) -> CaptureSceneResult:
    captures_dir = Path(captures_dir)
    manifest_path = captures_dir / "capture_scene_result.json"
    if not manifest_path.exists():
        raise FileNotFoundError(f"Ne postoji manifest datoteka: {manifest_path}")

    import json

    with open(manifest_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    view_results = [CaptureViewResult(**item) for item in data["view_results"]]
    return CaptureSceneResult(
        captures_dir=data["captures_dir"],
        required_views=int(data["required_views"]),
        views_captured=int(data["views_captured"]),
        pairs=data["pairs"],
        base_tcp_transform_paths=data["base_tcp_transform_paths"],
        view_results=view_results,
    )


if __name__ == "__main__":
    from config import cfg

    raise SystemExit(
        "Koristi capture_scene(cfg, ...), load_capture_scene_result(...), "
        "ili get_pairs_and_transforms_from_capture_result(...)."
    )