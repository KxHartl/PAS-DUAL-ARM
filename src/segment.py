# segment.py
from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterable

import numpy as np
import open3d as o3d
from PIL import Image, ImageDraw
from ultralytics import YOLO

from io_utils import (
    ensure_dir,
    load_point_cloud,
    load_rgb_image,
    make_point_cloud,
    point_cloud_to_image_grid,
    save_json,
    save_point_cloud,
    save_rgb_image,
)


@dataclass
class SegmentedInstance:
    instance_id: int
    class_name: str
    confidence: float
    bbox_xyxy: list[int]
    mask_area_px: int
    num_points_raw: int
    num_points_clean: int
    centroid_camera_xyz: list[float]
    segmented_pcd_path: str | None = None


@dataclass
class SegmentationResult:
    image_path: str
    pcd_path: str
    target_class: str
    overlay_path: str | None
    instances: list[SegmentedInstance]
    best_instance_id: int | None
    best_centroid_camera_xyz: list[float] | None
    fused_pcd_path: str | None


def _draw_overlay(
    image_rgb: np.ndarray,
    masks: np.ndarray,
    classes: np.ndarray,
    scores: np.ndarray,
    boxes: np.ndarray,
    class_names: dict[int, str] | list[str],
    target_class: str,
    mask_threshold: float,
) -> np.ndarray:
    img = Image.fromarray(image_rgb).convert("RGBA")
    draw = ImageDraw.Draw(img, "RGBA")

    for i in range(len(masks)):
        class_id = int(classes[i])
        class_name = class_names[class_id]
        if class_name != target_class:
            continue

        mask = masks[i] > mask_threshold
        ys, xs = np.where(mask)

        for x, y in zip(xs, ys):
            draw.point((int(x), int(y)), fill=(0, 0, 255, 90))

        x1, y1, x2, y2 = boxes[i].astype(int).tolist()
        draw.rectangle((x1, y1, x2, y2), outline=(0, 255, 0, 255), width=2)
        draw.text((x1, max(0, y1 - 18)), f"{class_name} {scores[i]:.2f}", fill=(0, 255, 0, 255))

    return np.array(img.convert("RGB"))


def _light_clean_instance_cloud(pcd: o3d.geometry.PointCloud, cfg: Any) -> o3d.geometry.PointCloud:
    if pcd.is_empty():
        return pcd

    points = np.asarray(pcd.points)

    valid = np.isfinite(points).all(axis=1) & (np.linalg.norm(points, axis=1) > 1e-9)

    zmin = getattr(cfg.point_cloud, "passthrough_min_z", None)
    zmax = getattr(cfg.point_cloud, "passthrough_max_z", None)
    if zmin is not None:
        valid &= points[:, 2] >= zmin
    if zmax is not None:
        valid &= points[:, 2] <= zmax

    idx = np.where(valid)[0]
    if len(idx) == 0:
        return o3d.geometry.PointCloud()

    clean = pcd.select_by_index(idx)

    if len(clean.points) >= max(10, getattr(cfg.point_cloud, "sor_nb_neighbors", 20)):
        clean, ind = clean.remove_statistical_outlier(
            nb_neighbors=getattr(cfg.point_cloud, "sor_nb_neighbors", 20),
            std_ratio=getattr(cfg.point_cloud, "sor_std_ratio", 2.0),
        )
        if len(ind) == 0:
            return pcd.select_by_index(idx)

    radius_nb = getattr(cfg.point_cloud, "radius_outlier_nb_points", 0)
    radius = getattr(cfg.point_cloud, "radius_outlier_radius", 0.0)
    if radius_nb and radius and len(clean.points) >= radius_nb:
        clean, ind = clean.remove_radius_outlier(
            nb_points=radius_nb,
            radius=radius,
        )
        if len(ind) == 0:
            return pcd.select_by_index(idx)

    return clean


def _choose_best_instance(instances: list[SegmentedInstance]) -> int | None:
    if not instances:
        return None

    best = max(
        instances,
        key=lambda inst: (
            inst.confidence,
            inst.num_points_clean,
            inst.mask_area_px,
        ),
    )
    return best.instance_id


def _fuse_instance_clouds(clean_clouds: list[o3d.geometry.PointCloud], cfg: Any) -> o3d.geometry.PointCloud:
    if not clean_clouds:
        return o3d.geometry.PointCloud()

    fused = o3d.geometry.PointCloud()
    for cloud in clean_clouds:
        if cloud.is_empty():
            continue
        fused += cloud

    if fused.is_empty():
        return fused

    voxel_size = max(float(getattr(cfg.point_cloud, "voxel_size", 0.0)), 0.003)
    if voxel_size > 0.0:
        fused = fused.voxel_down_sample(voxel_size)

    fused = _light_clean_instance_cloud(fused, cfg)
    return fused


def _extract_target_instances_from_pair(
    image_path: str | Path,
    image_rgb: np.ndarray,
    pcd: o3d.geometry.PointCloud,
    cfg: Any,
    model: YOLO,
) -> tuple[list[SegmentedInstance], np.ndarray | None, list[o3d.geometry.PointCloud]]:
    points_grid, _ = point_cloud_to_image_grid(pcd, image_rgb.shape)

    result = model(
        str(image_path),
        conf=cfg.target.confidence_threshold,
        verbose=False,
    )[0]

    if result.masks is None or len(result.boxes) == 0:
        return [], None, []

    masks = result.masks.data.cpu().numpy()
    classes = result.boxes.cls.cpu().numpy().astype(int)
    scores = result.boxes.conf.cpu().numpy()
    boxes = result.boxes.xyxy.cpu().numpy()
    class_names = result.names

    overlay_rgb = _draw_overlay(
        image_rgb=image_rgb,
        masks=masks,
        classes=classes,
        scores=scores,
        boxes=boxes,
        class_names=class_names,
        target_class=cfg.target.target_class,
        mask_threshold=cfg.target.mask_threshold,
    )

    image_colors = image_rgb.astype(np.float32) / 255.0
    valid_3d = np.isfinite(points_grid).all(axis=2) & (np.linalg.norm(points_grid, axis=2) > 1e-9)

    instances: list[SegmentedInstance] = []
    clean_clouds: list[o3d.geometry.PointCloud] = []

    local_instance_counter = 0

    for i in range(len(masks)):
        class_id = int(classes[i])
        class_name = class_names[class_id]
        confidence = float(scores[i])

        if class_name != cfg.target.target_class:
            continue

        mask = masks[i] > cfg.target.mask_threshold
        final_mask = mask & valid_3d

        raw_points = points_grid[final_mask]
        raw_colors = image_colors[final_mask]

        if len(raw_points) < cfg.target.min_mask_points:
            continue

        raw_pcd = make_point_cloud(raw_points, raw_colors)
        clean_pcd = _light_clean_instance_cloud(raw_pcd, cfg)
        clean_points = np.asarray(clean_pcd.points)

        if len(clean_points) < cfg.target.min_mask_points:
            continue

        centroid = np.mean(clean_points, axis=0)
        bbox_xyxy = boxes[i].astype(int).tolist()

        instances.append(
            SegmentedInstance(
                instance_id=local_instance_counter,
                class_name=class_name,
                confidence=confidence,
                bbox_xyxy=bbox_xyxy,
                mask_area_px=int(final_mask.sum()),
                num_points_raw=int(len(raw_points)),
                num_points_clean=int(len(clean_points)),
                centroid_camera_xyz=centroid.astype(float).tolist(),
                segmented_pcd_path=None,
            )
        )
        clean_clouds.append(clean_pcd)
        local_instance_counter += 1

    return instances, overlay_rgb, clean_clouds


def segment_single_pair(
    image_path: str | Path,
    pcd_path: str | Path,
    cfg: Any,
    model: YOLO | None = None,
    output_dir: str | Path | None = None,
) -> SegmentationResult:
    image_path = Path(image_path)
    pcd_path = Path(pcd_path)

    image_rgb = load_rgb_image(image_path)
    pcd = load_point_cloud(pcd_path)

    if model is None:
        model = YOLO(str(cfg.paths.segmentation_model))

    output_dir = Path(output_dir) if output_dir is not None else Path(cfg.paths.output_dir) / "segment"
    ensure_dir(output_dir)

    instances, overlay_rgb, clean_clouds = _extract_target_instances_from_pair(
        image_path=image_path,
        image_rgb=image_rgb,
        pcd=pcd,
        cfg=cfg,
        model=model,
    )

    overlay_path = None
    if overlay_rgb is not None:
        overlay_path = save_rgb_image(output_dir / f"{image_path.stem}_overlay.png", overlay_rgb)

    for inst, inst_pcd in zip(instances, clean_clouds):
        if getattr(cfg.debug, "save_segmented_clouds", True):
            pcd_path_out = output_dir / f"{image_path.stem}_instance_{inst.instance_id:02d}.pcd"
            save_point_cloud(pcd_path_out, inst_pcd)
            inst.segmented_pcd_path = str(pcd_path_out)

    best_instance_id = _choose_best_instance(instances)
    best_centroid = None
    if best_instance_id is not None:
        best_centroid = next(
            inst.centroid_camera_xyz for inst in instances if inst.instance_id == best_instance_id
        )

    fused_pcd_path = None
    fused_cloud = _fuse_instance_clouds(clean_clouds, cfg)
    if not fused_cloud.is_empty():
        fused_pcd_path = output_dir / f"{image_path.stem}_target_fused.pcd"
        save_point_cloud(fused_pcd_path, fused_cloud)
        if best_centroid is None:
            best_centroid = np.mean(np.asarray(fused_cloud.points), axis=0).astype(float).tolist()

    out = SegmentationResult(
        image_path=str(image_path),
        pcd_path=str(pcd_path),
        target_class=cfg.target.target_class,
        overlay_path=str(overlay_path) if overlay_path is not None else None,
        instances=instances,
        best_instance_id=best_instance_id,
        best_centroid_camera_xyz=best_centroid,
        fused_pcd_path=str(fused_pcd_path) if fused_pcd_path is not None else None,
    )

    save_json(output_dir / f"{image_path.stem}_segment_result.json", asdict(out))
    return out


def segment_multiple_views(
    pairs: Iterable[tuple[str | Path, str | Path]],
    cfg: Any,
    output_dir: str | Path | None = None,
) -> list[SegmentationResult]:
    model = YOLO(str(cfg.paths.segmentation_model))
    results: list[SegmentationResult] = []

    base_dir = Path(output_dir) if output_dir is not None else Path(cfg.paths.output_dir) / "segment"

    for idx, (image_path, pcd_path) in enumerate(pairs):
        subdir = base_dir / f"view_{idx:02d}"
        result = segment_single_pair(
            image_path=image_path,
            pcd_path=pcd_path,
            cfg=cfg,
            model=model,
            output_dir=subdir,
        )
        results.append(result)

    return results


if __name__ == "__main__":
    from config import cfg

    # Primjer:
    # result = segment_single_pair(
    #     image_path="data/captures/view_00/rgb.png",
    #     pcd_path="data/captures/view_00/cloud.pcd",
    #     cfg=cfg,
    # )
    # print(result)

    raise SystemExit(
        "Koristi segment_single_pair(...) ili segment_multiple_views(...) iz drugog modula."
    )
