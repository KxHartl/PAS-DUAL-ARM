# main.py
from __future__ import annotations

from dataclasses import asdict
from pathlib import Path
from typing import Any
import time

from config import cfg
from io_utils import ensure_dir, save_json
from capture_scene import (
    capture_scene,
    get_pairs_and_transforms_from_capture_result,
    load_capture_scene_result,
)
from reconstruct_scene import reconstruct_scene_from_pairs
from estimate_pick_pose import estimate_pick_pose_from_reconstruction
from trajectory_planner import plan_pick_place_trajectory
from ur_executor import execute_pick_place_program


def _timestamp() -> str:
    return time.strftime("%Y%m%d_%H%M%S")


def _log(msg: str, cfg: Any) -> None:
    if bool(getattr(cfg.runtime, "verbose", True)):
        print(msg)


def run_pipeline(
    cfg: Any,
    run_name: str | None = None,
    capture_new_scene: bool = True,
    existing_captures_dir: str | Path | None = None,
    move_robot_for_capture: bool = True,
    execute_pick_place: bool | None = None,
    invert_cam_tcp: bool = False,
) -> dict[str, Any]:
    """
    Glavni orkestrator cijelog sustava.

    Redoslijed:
    1) capture_scene
    2) reconstruct_scene_from_pairs
    3) estimate_pick_pose_from_reconstruction
    4) plan_pick_place_trajectory
    5) execute_pick_place_program

    Ako je cfg.runtime.offline_mode=True, pick-and-place program se po defaultu
    samo generira i sprema, bez slanja robotu.
    """
    run_name = run_name or f"run_{_timestamp()}"
    run_dir = ensure_dir(Path(cfg.paths.output_dir) / run_name)

    _log(f"[MAIN] Pokrecem pipeline: {run_name}", cfg)
    _log(f"[MAIN] Output direktorij: {run_dir}", cfg)

    if execute_pick_place is None:
        execute_pick_place = not bool(getattr(cfg.runtime, "offline_mode", True))

    # 1) CAPTURE
    if capture_new_scene:
        _log("[MAIN] 1/5 Capture scene...", cfg)
        capture_result = capture_scene(
            cfg=cfg,
            output_dir=run_dir / "captures",
            move_robot=move_robot_for_capture,
        )
    else:
        if existing_captures_dir is None:
            raise ValueError(
                "Ako je capture_new_scene=False, moraš zadati existing_captures_dir."
            )
        _log(f"[MAIN] 1/5 Ucitavam postojeci capture iz: {existing_captures_dir}", cfg)
        capture_result = load_capture_scene_result(existing_captures_dir)

    pairs, base_tcp_transforms = get_pairs_and_transforms_from_capture_result(capture_result)
    _log(f"[MAIN] Capture gotov. Broj viewova: {capture_result.views_captured}", cfg)

    # 2) RECONSTRUCT + SEGMENT
    _log("[MAIN] 2/5 Reconstruct scene...", cfg)
    reconstruction_result = reconstruct_scene_from_pairs(
        pairs=pairs,
        base_tcp_transforms=base_tcp_transforms,
        cfg=cfg,
        segment_output_dir=run_dir / "segment",
        reconstruct_output_dir=run_dir / "reconstruct_scene",
        invert_cam_tcp=invert_cam_tcp,
    )
    _log(
        f"[MAIN] Rekonstrukcija gotova. Pronadjeno instanci: {len(reconstruction_result.instances)}",
        cfg,
    )
    for inst in reconstruction_result.instances:
        _log(f"  - Instance {inst.instance_id}: centroid={inst.centroid_base_xyz}", cfg)

    # 3) ESTIMATE PICK POSE
    _log("[MAIN] 3/5 Estimate pick pose...", cfg)
    pick_pose_results = estimate_pick_pose_from_reconstruction(
        reconstruction_result=reconstruction_result,
        cfg=cfg,
        output_dir=run_dir / "estimate_pick_pose",
    )
    if not pick_pose_results:
        raise RuntimeError("Nijedna pick poza nije generirana.")
        
    _log(f"[MAIN] Broj generiranih PICK poza: {len(pick_pose_results)}", cfg)
    for i, res in enumerate(pick_pose_results):
        _log(f"  - Pick {i}: {res.pick_pose}", cfg)

    # Odabiremo PRVU pozu za planiranje i izvrsavanje
    target_pick_pose_result = pick_pose_results[0]
    _log(f"[MAIN] Odabran target za pick: Instance {reconstruction_result.instances[0].instance_id}", cfg)

    # 4) PLAN TRAJECTORY
    _log("[MAIN] 4/5 Plan trajectory...", cfg)
    trajectory_plan = plan_pick_place_trajectory(
        pick_pose_result=target_pick_pose_result,
        cfg=cfg,
        output_dir=run_dir / "trajectory_planner",
    )
    _log(
        f"[MAIN] Trajektorija gotova. Segmenata: {len(trajectory_plan.segments)}, "
        f"ukupno uzoraka: {trajectory_plan.total_samples}",
        cfg,
    )

    # 5) EXECUTE PICK-AND-PLACE
    _log("[MAIN] 5/5 Build/execute pick-and-place UR program...", cfg)
    program_path = execute_pick_place_program(
        trajectory_plan=trajectory_plan,
        cfg=cfg,
        save_program=True,
        send_to_robot=execute_pick_place,
    )
    if execute_pick_place:
        _log("[MAIN] Pick-and-place program je poslan robotu.", cfg)
    else:
        _log("[MAIN] Offline mode / no-send: program je samo spremljen.", cfg)

    summary = {
        "run_name": run_name,
        "run_dir": str(run_dir),
        "capture_result": asdict(capture_result),
        "reconstruction_result": asdict(reconstruction_result),
        "pick_pose_results": [asdict(r) for r in pick_pose_results],
        "trajectory_plan": asdict(trajectory_plan),
        "pick_place_program_path": str(program_path) if program_path is not None else None,
        "execute_pick_place": bool(execute_pick_place),
        "invert_cam_tcp": bool(invert_cam_tcp),
    }

    save_json(run_dir / "pipeline_summary.json", summary)
    _log("[MAIN] Pipeline zavrsen.", cfg)

    return summary


if __name__ == "__main__":
    # Zadani sigurni način rada:
    # - ako je offline_mode=True -> program se NE šalje robotu
    # - ako je offline_mode=False -> program se šalje robotu
    summary = run_pipeline(
        cfg=cfg,
        capture_new_scene=True,
        move_robot_for_capture=True,
        execute_pick_place=None,
        invert_cam_tcp=False,
    )

    print("\n=== PIPELINE SUMMARY ===")
    print(f"Run dir: {summary['run_dir']}")
    print(f"Target: {summary['reconstruction_result']['target_class']}")
    print(f"Number of instances: {len(summary['reconstruction_result']['instances'])}")
    if summary['reconstruction_result']['instances']:
        print(f"First centroid base xyz: {summary['reconstruction_result']['instances'][0]['centroid_base_xyz']}")
    print(f"Selected PICK pose: {summary['pick_pose_results'][0]['pick_pose']}")
    print(f"Trajectory samples: {summary['trajectory_plan']['total_samples']}")
    print(f"Program path: {summary['pick_place_program_path']}")