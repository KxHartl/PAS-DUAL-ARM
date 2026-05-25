# reconstruct_scene.py
from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterable

import numpy as np
import open3d as o3d

from io_utils import (
    ensure_dir,
    load_npy_matrix,
    load_point_cloud,
    make_point_cloud,
    save_json,
    save_point_cloud,
)
from segment import SegmentationResult, segment_multiple_views


@dataclass
class ReconstructedInstance:
    instance_id: int
    centroid_base_xyz: list[float]
    pcd_path: str
    num_points: int


@dataclass
class ReconstructionResult:
    target_class: str
    views_used: int
    instances: list[ReconstructedInstance]
    global_pcd_path: str | None


def _resolve_tcp_from_cam(cfg: Any, invert_cam_tcp: bool = False) -> np.ndarray:
    """
    Preferira eksplicitni T_tcp_from_cam ako postoji.
    Ako ne postoji, koristi cfg.paths.t_cam_from_tcp.
    Ako znaš da je u toj datoteci zapravo spremljen T_cam_from_tcp,
    postavi invert_cam_tcp=True.
    """
    tcp_from_cam_path = Path(cfg.paths.t_tcp_from_cam)
    cam_from_tcp_path = Path(cfg.paths.t_cam_from_tcp)

    if tcp_from_cam_path.exists():
        return load_npy_matrix(tcp_from_cam_path, expected_shape=(4, 4))

    T = load_npy_matrix(cam_from_tcp_path, expected_shape=(4, 4))
    if invert_cam_tcp:
        return np.linalg.inv(T)
    return T


def _voxel_downsample_if_needed(pcd: o3d.geometry.PointCloud, voxel_size: float) -> o3d.geometry.PointCloud:
    if pcd.is_empty():
        return pcd
    if voxel_size is None or voxel_size <= 0.0:
        return pcd
    return pcd.voxel_down_sample(voxel_size)


def _light_post_merge_clean(pcd: o3d.geometry.PointCloud, cfg: Any) -> o3d.geometry.PointCloud:
    if pcd.is_empty():
        return pcd

    clean = pcd
    voxel_size = getattr(cfg.point_cloud, "voxel_size", 0.0)
    if voxel_size and voxel_size > 0.0:
        clean = clean.voxel_down_sample(voxel_size)

    if len(clean.points) >= max(10, getattr(cfg.point_cloud, "sor_nb_neighbors", 20)):
        clean, ind = clean.remove_statistical_outlier(
            nb_neighbors=getattr(cfg.point_cloud, "sor_nb_neighbors", 20),
            std_ratio=getattr(cfg.point_cloud, "sor_std_ratio", 2.0),
        )
        if len(ind) == 0:
            return pcd

    radius_nb = getattr(cfg.point_cloud, "radius_outlier_nb_points", 0)
    radius = getattr(cfg.point_cloud, "radius_outlier_radius", 0.0)
    if radius_nb and radius and len(clean.points) >= radius_nb:
        clean, ind = clean.remove_radius_outlier(
            nb_points=radius_nb,
            radius=radius,
        )
        if len(ind) == 0:
            return clean

    return clean


def _transform_cloud(pcd: o3d.geometry.PointCloud, T: np.ndarray) -> o3d.geometry.PointCloud:
    out = o3d.geometry.PointCloud(pcd)
    out.transform(T)
    return out


def reconstruct_scene_from_results(
    segment_results: Iterable[SegmentationResult],
    base_tcp_transforms: Iterable[np.ndarray],
    cfg: Any,
    output_dir: str | Path | None = None,
    invert_cam_tcp: bool = False,
) -> ReconstructionResult:
    segment_results = list(segment_results)
    base_tcp_transforms = [np.asarray(T, dtype=np.float64) for T in base_tcp_transforms]

    if len(segment_results) == 0:
        raise ValueError("segment_results je prazan.")
    if len(segment_results) != len(base_tcp_transforms):
        raise ValueError("Broj segment_results i broj base_tcp_transforms mora biti isti.")

    output_dir = Path(output_dir) if output_dir is not None else Path(cfg.paths.output_dir) / "reconstruct_scene"
    ensure_dir(output_dir)

    T_tcp_cam = _resolve_tcp_from_cam(cfg, invert_cam_tcp=invert_cam_tcp)

    all_transformed_points = []
    all_transformed_colors = []

    for idx, (seg_result, T_base_tcp) in enumerate(zip(segment_results, base_tcp_transforms)):
        # Matematicko stichanje: P_base = T_base_tcp * T_tcp_cam * P_cam
        # Brute-force testom smo potvrdili da je Tb @ H tocna formula
        T_base_cam = T_base_tcp @ T_tcp_cam
        
        for inst in seg_result.instances:
            if not inst.segmented_pcd_path:
                continue
            
            pcd_cam = load_point_cloud(inst.segmented_pcd_path)
            if pcd_cam.is_empty():
                continue
                
            pcd_base = _transform_cloud(pcd_cam, T_base_cam)
            all_transformed_points.append(np.asarray(pcd_base.points))
            all_transformed_colors.append(np.asarray(pcd_base.colors))

    if not all_transformed_points:
        raise RuntimeError("Nema detektiranih instanci za rekonstrukciju.")

    global_pcd = o3d.geometry.PointCloud()
    global_pcd.points = o3d.utility.Vector3dVector(np.concatenate(all_transformed_points, axis=0))
    global_pcd.colors = o3d.utility.Vector3dVector(np.concatenate(all_transformed_colors, axis=0))
    
    global_pcd = _light_post_merge_clean(global_pcd, cfg)
    
    global_pcd_path = output_dir / "global_merged.pcd"
    save_point_cloud(global_pcd_path, global_pcd)

    # DBSCAN Clustering za razdvajanje pojedinacnih vocaka
    eps = float(getattr(cfg.point_cloud, "cluster_tolerance", 0.02))
    min_points = int(getattr(cfg.point_cloud, "min_cluster_size", 50))
    
    labels = np.array(global_pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=False))
    max_label = labels.max()
    
    reconstructed_instances = []
    
    for i in range(max_label + 1):
        indices = np.where(labels == i)[0]
        cluster_pcd = global_pcd.select_by_index(indices)
        
        # Clean cluster
        cluster_pcd = _light_post_merge_clean(cluster_pcd, cfg)
        if cluster_pcd.is_empty():
            continue
            
        centroid = np.mean(np.asarray(cluster_pcd.points), axis=0)
        
        pcd_path = output_dir / f"instance_{i:02d}.pcd"
        save_point_cloud(pcd_path, cluster_pcd)
        
        reconstructed_instances.append(
            ReconstructedInstance(
                instance_id=i,
                centroid_base_xyz=centroid.tolist(),
                pcd_path=str(pcd_path),
                num_points=len(cluster_pcd.points)
            )
        )

    result = ReconstructionResult(
        target_class=cfg.target.target_class,
        views_used=len(segment_results),
        instances=reconstructed_instances,
        global_pcd_path=str(global_pcd_path)
    )

    save_json(output_dir / "reconstruction_result.json", asdict(result))
    return result


def reconstruct_scene_from_pairs(
    pairs: Iterable[tuple[str | Path, str | Path]],
    base_tcp_transforms: Iterable[np.ndarray],
    cfg: Any,
    segment_output_dir: str | Path | None = None,
    reconstruct_output_dir: str | Path | None = None,
    invert_cam_tcp: bool = False,
) -> ReconstructionResult:
    pairs = list(pairs)
    segment_output_dir = (
        Path(segment_output_dir)
        if segment_output_dir is not None
        else Path(cfg.paths.output_dir) / "segment"
    )

    segment_results = segment_multiple_views(
        pairs=pairs,
        cfg=cfg,
        output_dir=segment_output_dir,
    )

    return reconstruct_scene_from_results(
        segment_results=segment_results,
        base_tcp_transforms=base_tcp_transforms,
        cfg=cfg,
        output_dir=reconstruct_output_dir,
        invert_cam_tcp=invert_cam_tcp,
    )


if __name__ == "__main__":
    from config import cfg

    raise SystemExit(
        "Koristi reconstruct_scene_from_results(...) ili reconstruct_scene_from_pairs(...)."
    )