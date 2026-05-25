# trajectory_planner.py
from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterable

import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R, Slerp

from io_utils import ensure_dir, save_json


@dataclass
class SegmentTrajectory:
    name: str
    start_pose: list[float]
    end_pose: list[float]
    duration: float
    dt: float
    num_samples: int
    positions_xyz: list[list[float]]
    orientations_rvec: list[list[float]]
    poses: list[list[float]]


@dataclass
class TrajectoryPlan:
    target_class: str
    reference_frame: str
    waypoint_names: list[str]
    waypoints: list[list[float]]
    total_duration: float
    total_samples: int
    servoj_dt: float
    segments: list[SegmentTrajectory]
    full_poses: list[list[float]]
    urscript_commands: list[str]
    urscript_txt_path: str | None


def _as_pose6(vec: Any, name: str) -> np.ndarray:
    arr = np.asarray(vec, dtype=np.float64).reshape(-1)
    if arr.shape != (6,):
        raise ValueError(f"{name} mora imati 6 elemenata [x, y, z, rx, ry, rz], dobiveno {arr.shape}.")
    return arr


def _quintic_time_scaling(tau: np.ndarray) -> np.ndarray:
    return 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5


def _segment_duration(start_pose: np.ndarray, end_pose: np.ndarray, cfg: Any) -> float:
    p_i = start_pose[:3]
    p_f = end_pose[:3]

    R_i = R.from_rotvec(start_pose[3:])
    R_f = R.from_rotvec(end_pose[3:])

    D = float(np.linalg.norm(p_f - p_i))
    alpha = float(np.linalg.norm((R_i.inv() * R_f).as_rotvec()))

    vmax = float(cfg.trajectory.vmax)
    amax = float(cfg.trajectory.amax)
    omega_max = float(cfg.trajectory.omega_max)
    minimum_segment_time = float(cfg.trajectory.minimum_segment_time)

    if getattr(cfg.trajectory, "use_quintic", True):
        k_v = 1.875
        k_a = 5.77
    else:
        k_v = 1.5
        k_a = 6.0

    t_vel = k_v * D / vmax if vmax > 0.0 else 0.0
    t_acc = np.sqrt(k_a * D / amax) if amax > 0.0 and D > 0.0 else 0.0
    t_rot = k_v * alpha / omega_max if omega_max > 0.0 else 0.0

    return float(max(t_vel, t_acc, t_rot, minimum_segment_time))


def _sample_segment(
    start_pose: np.ndarray,
    end_pose: np.ndarray,
    duration: float,
    num_samples: int,
) -> tuple[np.ndarray, np.ndarray]:
    if num_samples < 2:
        raise ValueError("num_samples mora biti >= 2.")

    t = np.linspace(0.0, duration, num_samples)
    tau = t / duration if duration > 0.0 else np.zeros_like(t)
    s = _quintic_time_scaling(tau)

    p_i = start_pose[:3]
    p_f = end_pose[:3]
    p = p_i[None, :] + (p_f - p_i)[None, :] * s[:, None]

    R_i = R.from_rotvec(start_pose[3:])
    R_f = R.from_rotvec(end_pose[3:])
    slerp = Slerp([0.0, 1.0], R.concatenate([R_i, R_f]))
    R_path = slerp(s)
    rvec = R_path.as_rotvec()

    poses = np.hstack([p, rvec])
    return t, poses


def _make_servoj_command(pose: np.ndarray, dt: float, cfg: Any) -> str:
    x, y, z, rx, ry, rz = pose.tolist()
    lookahead = float(cfg.robot.servoj_lookahead_time)
    gain = int(cfg.robot.servoj_gain)

    return (
        f"servoj(get_inverse_kin(p[{x:.6f}, {y:.6f}, {z:.6f}, "
        f"{rx:.6f}, {ry:.6f}, {rz:.6f}]), "
        f"t={dt:.6f}, lookahead_time={lookahead:.3f}, gain={gain})"
    )


def _save_trajectory_txt(
    urscript_commands: Iterable[str],
    output_dir: Path,
    filename: str = "trajectory_taskspace.txt",
) -> Path:
    ensure_dir(output_dir)
    txt_path = output_dir / filename
    with open(txt_path, "w", encoding="utf-8") as f:
        for line in urscript_commands:
            f.write(line.rstrip() + "\n")
    return txt_path


def _plot_trajectory(
    full_poses: np.ndarray,
    waypoint_names: list[str],
    waypoints: np.ndarray,
    output_dir: Path,
    filename: str = "trajectory_taskspace.png",
) -> Path:
    ensure_dir(output_dir)

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection="3d")

    ax.plot(full_poses[:, 0], full_poses[:, 1], full_poses[:, 2], linewidth=2, label="TCP putanja")
    ax.scatter(waypoints[:, 0], waypoints[:, 1], waypoints[:, 2], c="red", s=60, label="glavne tocke")

    for name, pose in zip(waypoint_names, waypoints):
        ax.text(pose[0], pose[1], pose[2], name, fontsize=8)

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.set_title("Task-space pick-and-place trajektorija")
    ax.grid(True)
    ax.legend()

    plt.tight_layout()
    plot_path = output_dir / filename
    fig.savefig(plot_path, dpi=180)
    plt.close(fig)
    return plot_path


def plan_pick_place_trajectory(
    pick_pose_result: Any,
    cfg: Any,
    output_dir: str | Path | None = None,
) -> TrajectoryPlan:
    output_dir = Path(output_dir) if output_dir is not None else Path(cfg.paths.output_dir) / "trajectory_planner"
    ensure_dir(output_dir)

    home_pose = _as_pose6(pick_pose_result.home_pose, "pick_pose_result.home_pose")
    approach_pick_pose = _as_pose6(pick_pose_result.approach_pick_pose, "pick_pose_result.approach_pick_pose")
    pick_pose = _as_pose6(pick_pose_result.pick_pose, "pick_pose_result.pick_pose")
    approach_place_pose = _as_pose6(pick_pose_result.approach_place_pose, "pick_pose_result.approach_place_pose")
    place_pose = _as_pose6(pick_pose_result.place_pose, "pick_pose_result.place_pose")

    waypoint_names = [
        "HOME",
        "APPROACH_PICK",
        "PICK",
        "APPROACH_PICK_RETURN",
        "APPROACH_PLACE",
        "PLACE",
        "APPROACH_PLACE_RETURN",
        "HOME_RETURN",
    ]
    waypoints = [
        home_pose,
        approach_pick_pose,
        pick_pose,
        approach_pick_pose.copy(),
        approach_place_pose,
        place_pose,
        approach_place_pose.copy(),
        home_pose.copy(),
    ]

    segment_names = [
        "HOME_to_APPROACH_PICK",
        "APPROACH_PICK_to_PICK",
        "PICK_to_APPROACH_PICK",
        "APPROACH_PICK_to_APPROACH_PLACE",
        "APPROACH_PLACE_to_PLACE",
        "PLACE_to_APPROACH_PLACE",
        "APPROACH_PLACE_to_HOME",
    ]

    samples_per_segment = int(cfg.trajectory.samples_per_segment)
    if samples_per_segment < 2:
        raise ValueError("cfg.trajectory.samples_per_segment mora biti >= 2.")

    segments: list[SegmentTrajectory] = []
    full_pose_list: list[np.ndarray] = []
    urscript_commands: list[str] = []
    total_duration = 0.0

    for idx, seg_name in enumerate(segment_names):
        start_pose = waypoints[idx]
        end_pose = waypoints[idx + 1]

        duration = _segment_duration(start_pose, end_pose, cfg)
        t, poses = _sample_segment(
            start_pose=start_pose,
            end_pose=end_pose,
            duration=duration,
            num_samples=samples_per_segment,
        )
        dt = float(t[1] - t[0]) if len(t) >= 2 else duration

        poses_for_full = poses if idx == 0 else poses[1:]
        full_pose_list.extend([row.copy() for row in poses_for_full])

        segments.append(
            SegmentTrajectory(
                name=seg_name,
                start_pose=start_pose.astype(float).tolist(),
                end_pose=end_pose.astype(float).tolist(),
                duration=float(duration),
                dt=float(dt),
                num_samples=int(len(poses)),
                positions_xyz=poses[:, :3].astype(float).tolist(),
                orientations_rvec=poses[:, 3:].astype(float).tolist(),
                poses=poses.astype(float).tolist(),
            )
        )
        total_duration += duration

        commands_segment = poses if idx == 0 else poses[1:]
        for pose in commands_segment:
            urscript_commands.append(_make_servoj_command(pose, dt, cfg))

    full_poses = np.asarray(full_pose_list, dtype=np.float64)
    servoj_dt = float(segments[0].dt) if len(segments) > 0 else 0.0

    urscript_txt_path = None
    if getattr(cfg.debug, "save_trajectory_txt", True):
        urscript_txt_path = _save_trajectory_txt(
            urscript_commands=urscript_commands,
            output_dir=output_dir,
        )

    if getattr(cfg.debug, "save_trajectory_plot", True) and len(full_poses) > 0:
        _plot_trajectory(
            full_poses=full_poses,
            waypoint_names=waypoint_names,
            waypoints=np.asarray(waypoints, dtype=np.float64),
            output_dir=output_dir,
        )

    result = TrajectoryPlan(
        target_class=getattr(pick_pose_result, "target_class", cfg.target.target_class),
        reference_frame=getattr(pick_pose_result, "reference_frame", "base"),
        waypoint_names=waypoint_names,
        waypoints=[pose.astype(float).tolist() for pose in waypoints],
        total_duration=float(total_duration),
        total_samples=int(len(full_poses)),
        servoj_dt=servoj_dt,
        segments=segments,
        full_poses=full_poses.astype(float).tolist(),
        urscript_commands=urscript_commands,
        urscript_txt_path=str(urscript_txt_path) if urscript_txt_path is not None else None,
    )

    save_json(output_dir / "trajectory_plan.json", asdict(result))
    return result


if __name__ == "__main__":
    from config import cfg

    raise SystemExit(
        "Koristi plan_pick_place_trajectory(pick_pose_result, cfg, output_dir=...)."
    )