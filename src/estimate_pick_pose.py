# estimate_pick_pose.py
from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np

from io_utils import ensure_dir, save_json
from reconstruct_scene import ReconstructionResult


@dataclass
class PickPoseResult:
    target_class: str
    reference_frame: str
    centroid_base_xyz: list[float]
    pick_orientation_rvec: list[float]
    home_pose: list[float]
    approach_pick_pose: list[float]
    pick_pose: list[float]
    approach_place_pose: list[float]
    place_pose: list[float]


def _as_xyz(vec: Any, name: str) -> np.ndarray:
    arr = np.asarray(vec, dtype=np.float64).reshape(-1)
    if arr.shape != (3,):
        raise ValueError(f"{name} mora imati 3 elementa, dobiveno {arr.shape}.")
    return arr


def _as_pose6(vec: Any, name: str) -> np.ndarray:
    arr = np.asarray(vec, dtype=np.float64).reshape(-1)
    if arr.shape != (6,):
        raise ValueError(f"{name} mora imati 6 elemenata [x, y, z, rx, ry, rz], dobiveno {arr.shape}.")
    return arr


def _compose_pose(xyz: np.ndarray, rvec: np.ndarray) -> list[float]:
    xyz = _as_xyz(xyz, "xyz")
    rvec = _as_xyz(rvec, "rvec")
    return np.hstack([xyz, rvec]).astype(float).tolist()


def _load_reconstruction_result_json(path: str | Path) -> dict[str, Any]:
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"Ne postoji reconstruction_result.json: {path}")

    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    if not isinstance(data, dict):
        raise ValueError(f"Očekivao sam JSON objekt u {path}.")
    return data


def estimate_pick_pose_from_centroid(
    centroid_base_xyz: list[float] | tuple[float, float, float] | np.ndarray,
    cfg: Any,
    output_dir: str | Path | None = None,
    target_class: str | None = None,
) -> PickPoseResult:
    centroid = _as_xyz(centroid_base_xyz, "centroid_base_xyz")

    home_pose = _as_pose6(cfg.pick_place.home_pose, "cfg.pick_place.home_pose")
    place_pose_cfg = _as_pose6(cfg.pick_place.place_pose, "cfg.pick_place.place_pose")
    pick_rvec = _as_xyz(cfg.robot.tcp_orientation_rvec, "cfg.robot.tcp_orientation_rvec")
    place_rvec = place_pose_cfg[3:].copy()

    approach_offset_z = float(getattr(cfg.pick_place, "approach_offset_z", 0.0))
    grasp_offset_z = float(getattr(cfg.pick_place, "grasp_offset_z", 0.0))
    place_offset_z = float(getattr(cfg.pick_place, "place_offset_z", 0.0))

    pick_xyz = centroid.copy()
    pick_xyz[2] += grasp_offset_z

    approach_pick_xyz = pick_xyz.copy()
    approach_pick_xyz[2] += approach_offset_z

    place_xyz = place_pose_cfg[:3].copy()
    place_xyz[2] += place_offset_z

    approach_place_xyz = place_xyz.copy()
    approach_place_xyz[2] += approach_offset_z

    result = PickPoseResult(
        target_class=target_class if target_class is not None else cfg.target.target_class,
        reference_frame="base",
        centroid_base_xyz=centroid.astype(float).tolist(),
        pick_orientation_rvec=pick_rvec.astype(float).tolist(),
        home_pose=home_pose.astype(float).tolist(),
        approach_pick_pose=_compose_pose(approach_pick_xyz, pick_rvec),
        pick_pose=_compose_pose(pick_xyz, pick_rvec),
        approach_place_pose=_compose_pose(approach_place_xyz, place_rvec),
        place_pose=_compose_pose(place_xyz, place_rvec),
    )

    output_dir = Path(output_dir) if output_dir is not None else Path(cfg.paths.output_dir) / "estimate_pick_pose"
    ensure_dir(output_dir)
    save_json(output_dir / "pick_pose_result.json", asdict(result))

    return result


def estimate_pick_pose_from_reconstruction(
    reconstruction_result: ReconstructionResult,
    cfg: Any,
    output_dir: str | Path | None = None,
) -> list[PickPoseResult]:
    output_dir = Path(output_dir) if output_dir is not None else Path(cfg.paths.output_dir) / "estimate_pick_pose"
    ensure_dir(output_dir)
    
    results = []
    for instance in reconstruction_result.instances:
        res = estimate_pick_pose_from_centroid(
            centroid_base_xyz=instance.centroid_base_xyz,
            cfg=cfg,
            output_dir=output_dir / f"instance_{instance.instance_id:02d}",
            target_class=reconstruction_result.target_class,
        )
        results.append(res)
        
    save_json(output_dir / "multi_pick_pose_results.json", [asdict(r) for r in results])
    return results


def estimate_pick_pose_from_reconstruction_json(
    reconstruction_json_path: str | Path,
    cfg: Any,
    output_dir: str | Path | None = None,
) -> list[PickPoseResult]:
    data = _load_reconstruction_result_json(reconstruction_json_path)

    instances_data = data.get("instances", [])
    if not instances_data:
        # Fallback for old format
        centroid = data.get("centroid_base_xyz", None)
        if centroid is None:
             raise RuntimeError(f"U {reconstruction_json_path} ne postoje instance niti centroid_base_xyz.")
        instances_data = [{"instance_id": 0, "centroid_base_xyz": centroid}]

    target_class = data.get("target_class", cfg.target.target_class)
    
    output_dir = Path(output_dir) if output_dir is not None else Path(cfg.paths.output_dir) / "estimate_pick_pose"
    ensure_dir(output_dir)

    results = []
    for inst in instances_data:
        res = estimate_pick_pose_from_centroid(
            centroid_base_xyz=inst["centroid_base_xyz"],
            cfg=cfg,
            output_dir=output_dir / f"instance_{inst['instance_id']:02d}",
            target_class=target_class,
        )
        results.append(res)

    save_json(output_dir / "multi_pick_pose_results.json", [asdict(r) for r in results])
    return results


if __name__ == "__main__":
    from config import cfg

    raise SystemExit(
        "Koristi estimate_pick_pose_from_reconstruction(...), "
        "estimate_pick_pose_from_reconstruction_json(...) ili "
        "estimate_pick_pose_from_centroid(...)."
    )