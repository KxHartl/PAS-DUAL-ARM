# ur_executor.py
from __future__ import annotations

import socket
import time
from pathlib import Path
from typing import Any, Iterable

import numpy as np

from io_utils import ensure_dir
from trajectory_planner import TrajectoryPlan


def _as_pose6(vec: Any, name: str) -> np.ndarray:
    arr = np.asarray(vec, dtype=np.float64).reshape(-1)
    if arr.shape != (6,):
        raise ValueError(f"{name} mora imati 6 elemenata [x, y, z, rx, ry, rz], dobiveno {arr.shape}.")
    return arr


def _pose_to_urscript(pose: Any) -> str:
    pose = _as_pose6(pose, "pose")
    x, y, z, rx, ry, rz = pose.tolist()
    return f"p[{x:.6f}, {y:.6f}, {z:.6f}, {rx:.6f}, {ry:.6f}, {rz:.6f}]"


def _build_movej_command(pose: Any, cfg: Any) -> str:
    pose_str = _pose_to_urscript(pose)
    a = float(cfg.robot.movej_acc)
    v = float(cfg.robot.movej_vel)
    return f"movej(get_inverse_kin({pose_str}), a={a:.6f}, v={v:.6f})"


def _build_servoj_command(pose: Any, dt: float, cfg: Any) -> str:
    pose = _as_pose6(pose, "pose")
    x, y, z, rx, ry, rz = pose.tolist()
    lookahead = float(cfg.robot.servoj_lookahead_time)
    gain = int(cfg.robot.servoj_gain)

    return (
        f"servoj(get_inverse_kin(p[{x:.6f}, {y:.6f}, {z:.6f}, "
        f"{rx:.6f}, {ry:.6f}, {rz:.6f}]), "
        f"t={dt:.6f}, lookahead_time={lookahead:.3f}, gain={gain})"
    )


def _wrap_program(lines: list[str], program_name: str = "prog") -> str:
    body = [line.rstrip() for line in lines if str(line).strip()]
    program = [f"def {program_name}():"]
    program.extend([f"  {line}" for line in body])
    program.append("end")
    program.append(f"{program_name}()")
    program.append("")
    return "\n".join(program)


def _save_program(program: str, output_path: str | Path) -> Path:
    output_path = Path(output_path)
    ensure_dir(output_path.parent)
    with open(output_path, "w", encoding="utf-8") as f:
        f.write(program)
    return output_path


def _send_program(
    program: str,
    host: str,
    port: int,
    timeout_s: float = 5.0,
) -> None:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(timeout_s)
        s.connect((host, port))
        s.sendall(program.encode("utf-8"))


def _default_gripper_open_command(pulse_s: float = 0.5) -> str:
    lines = [
        "set_standard_digital_out(4, False)",
        "set_standard_digital_out(5, True)",
        f"sleep({pulse_s:.3f})",
        "set_standard_digital_out(4, False)",
        "set_standard_digital_out(5, False)",
    ]
    return "\n".join(lines)


def _default_gripper_close_command(pulse_s: float = 0.5) -> str:
    lines = [
        "set_standard_digital_out(5, False)",
        "set_standard_digital_out(4, True)",
        f"sleep({pulse_s:.3f})",
        "set_standard_digital_out(4, False)",
        "set_standard_digital_out(5, False)",
    ]
    return "\n".join(lines)


def _get_gripper_open_command(cfg: Any) -> str:
    custom = getattr(cfg.pick_place, "gripper_open_command", "")
    pulse_s = float(getattr(cfg.pick_place, "gripper_pulse_time", 0.5))
    return custom.strip() if isinstance(custom, str) and custom.strip() else _default_gripper_open_command(pulse_s)


def _get_gripper_close_command(cfg: Any) -> str:
    custom = getattr(cfg.pick_place, "gripper_close_command", "")
    pulse_s = float(getattr(cfg.pick_place, "gripper_pulse_time", 0.5))
    return custom.strip() if isinstance(custom, str) and custom.strip() else _default_gripper_close_command(pulse_s)


def _expand_multiline_urscript(cmd: str) -> list[str]:
    return [line.rstrip() for line in cmd.splitlines() if line.strip()]


def _segment_map(plan: TrajectoryPlan) -> dict[str, Any]:
    return {seg.name: seg for seg in plan.segments}


def _segment_servoj_lines(segment: Any, cfg: Any, skip_first_pose: bool = False) -> list[str]:
    poses = np.asarray(segment.poses, dtype=np.float64)
    if poses.ndim != 2 or poses.shape[1] != 6:
        raise ValueError(f"Segment {segment.name} nema validan shape za poses.")

    dt = float(segment.dt)
    start_idx = 1 if skip_first_pose else 0
    return [_build_servoj_command(pose, dt, cfg) for pose in poses[start_idx:]]


def build_capture_program(
    cfg: Any,
    capture_poses: list[list[float]] | None = None,
    dwell_s: float = 0.3,
) -> str:
    poses = capture_poses if capture_poses is not None else cfg.capture.capture_poses

    lines: list[str] = []
    for pose in poses:
        lines.append(_build_movej_command(pose, cfg))
        if dwell_s > 0.0:
            lines.append(f"sleep({float(dwell_s):.3f})")

    return _wrap_program(lines, program_name="capture_prog")


def execute_capture_program(
    cfg: Any,
    capture_poses: list[list[float]] | None = None,
    dwell_s: float = 0.3,
    save_program: bool = True,
    send_to_robot: bool | None = None,
) -> Path | None:
    output_dir = Path(cfg.paths.output_dir) / "ur_executor"
    ensure_dir(output_dir)

    program = build_capture_program(cfg=cfg, capture_poses=capture_poses, dwell_s=dwell_s)
    program_path = output_dir / "capture_program.script"

    if save_program:
        _save_program(program, program_path)

    if send_to_robot is None:
        send_to_robot = not bool(getattr(cfg.runtime, "offline_mode", True))

    if send_to_robot:
        _send_program(
            program=program,
            host=str(cfg.robot.robot_host),
            port=int(cfg.robot.robot_port_script),
        )

    return program_path if save_program else None


def build_pick_place_program(
    trajectory_plan: TrajectoryPlan,
    cfg: Any,
) -> str:
    segments = _segment_map(trajectory_plan)

    required_segment_names = [
        "HOME_to_APPROACH_PICK",
        "APPROACH_PICK_to_PICK",
        "PICK_to_APPROACH_PICK",
        "APPROACH_PICK_to_APPROACH_PLACE",
        "APPROACH_PLACE_to_PLACE",
        "PLACE_to_APPROACH_PLACE",
        "APPROACH_PLACE_to_HOME",
    ]
    missing = [name for name in required_segment_names if name not in segments]
    if missing:
        raise RuntimeError(
            f"TrajectoryPlan ne sadrži sve očekivane segmente. Nedostaju: {missing}"
        )

    lines: list[str] = []
    home_pose = trajectory_plan.waypoints[0]
    lines.append(_build_movej_command(home_pose, cfg))
    lines.append("sleep(0.200)")

    lines.extend(_segment_servoj_lines(segments["HOME_to_APPROACH_PICK"], cfg, skip_first_pose=False))

    if bool(getattr(cfg.pick_place, "open_gripper_before_pick", True)):
        lines.append("sleep(0.050)")
        lines.extend(_expand_multiline_urscript(_get_gripper_open_command(cfg)))
        lines.append("sleep(0.050)")

    lines.extend(_segment_servoj_lines(segments["APPROACH_PICK_to_PICK"], cfg, skip_first_pose=True))

    if bool(getattr(cfg.pick_place, "close_gripper_at_pick", True)):
        lines.append("sleep(0.050)")
        lines.extend(_expand_multiline_urscript(_get_gripper_close_command(cfg)))
        lines.append("sleep(0.050)")

    lines.extend(_segment_servoj_lines(segments["PICK_to_APPROACH_PICK"], cfg, skip_first_pose=True))
    lines.extend(_segment_servoj_lines(segments["APPROACH_PICK_to_APPROACH_PLACE"], cfg, skip_first_pose=True))
    lines.extend(_segment_servoj_lines(segments["APPROACH_PLACE_to_PLACE"], cfg, skip_first_pose=True))

    if bool(getattr(cfg.pick_place, "open_gripper_at_place", True)):
        lines.append("sleep(0.050)")
        lines.extend(_expand_multiline_urscript(_get_gripper_open_command(cfg)))
        lines.append("sleep(0.050)")

    lines.extend(_segment_servoj_lines(segments["PLACE_to_APPROACH_PLACE"], cfg, skip_first_pose=True))
    lines.extend(_segment_servoj_lines(segments["APPROACH_PLACE_to_HOME"], cfg, skip_first_pose=True))

    return _wrap_program(lines, program_name="pick_place_prog")


def execute_pick_place_program(
    trajectory_plan: TrajectoryPlan,
    cfg: Any,
    save_program: bool = True,
    send_to_robot: bool | None = None,
) -> Path | None:
    output_dir = Path(cfg.paths.output_dir) / "ur_executor"
    ensure_dir(output_dir)

    program = build_pick_place_program(trajectory_plan=trajectory_plan, cfg=cfg)
    program_path = output_dir / "pick_place_program.script"

    if save_program:
        _save_program(program, program_path)

    if send_to_robot is None:
        send_to_robot = not bool(getattr(cfg.runtime, "offline_mode", True))

    if send_to_robot:
        _send_program(
            program=program,
            host=str(cfg.robot.robot_host),
            port=int(cfg.robot.robot_port_script),
        )

    return program_path if save_program else None


if __name__ == "__main__":
    from config import cfg

    raise SystemExit(
        "Koristi build_capture_program(...), execute_capture_program(...), "
        "build_pick_place_program(...) ili execute_pick_place_program(...)."
    )