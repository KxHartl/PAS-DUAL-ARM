# io_utils
from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import open3d as o3d


def ensure_dir(path: str | Path) -> Path:
    path = Path(path)
    path.mkdir(parents=True, exist_ok=True)
    return path


def load_rgb_image(image_path: str | Path) -> np.ndarray:
    image_path = Path(image_path)
    image_bgr = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image_bgr is None:
        raise FileNotFoundError(f"Ne mogu ucitati sliku: {image_path}")
    image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
    return np.ascontiguousarray(image_rgb)


def save_rgb_image(image_path: str | Path, image_rgb: np.ndarray) -> Path:
    image_path = Path(image_path)
    ensure_dir(image_path.parent)
    image_bgr = cv2.cvtColor(np.ascontiguousarray(image_rgb), cv2.COLOR_RGB2BGR)
    ok = cv2.imwrite(str(image_path), image_bgr)
    if not ok:
        raise IOError(f"Ne mogu spremiti sliku: {image_path}")
    return image_path


def load_point_cloud(pcd_path: str | Path) -> o3d.geometry.PointCloud:
    pcd_path = Path(pcd_path)
    pcd = o3d.io.read_point_cloud(str(pcd_path))
    if pcd.is_empty():
        raise ValueError(f"Point cloud je prazan ili ga nije moguce ucitati: {pcd_path}")
    return pcd


def save_point_cloud(pcd_path: str | Path, pcd: o3d.geometry.PointCloud) -> Path:
    pcd_path = Path(pcd_path)
    ensure_dir(pcd_path.parent)
    ok = o3d.io.write_point_cloud(str(pcd_path), pcd)
    if not ok:
        raise IOError(f"Ne mogu spremiti point cloud: {pcd_path}")
    return pcd_path


def load_npy_matrix(path: str | Path, expected_shape: tuple[int, ...] | None = None) -> np.ndarray:
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"Datoteka ne postoji: {path}")

    arr = np.load(path)
    if expected_shape is not None and arr.shape != expected_shape:
        raise ValueError(
            f"Kriva dimenzija za {path}. Ocekivano {expected_shape}, dobiveno {arr.shape}."
        )
    return arr


def point_cloud_to_image_grid(
    pcd: o3d.geometry.PointCloud,
    image_shape: tuple[int, ...],
) -> tuple[np.ndarray, np.ndarray]:
    h, w = image_shape[:2]

    points = np.asarray(pcd.points, dtype=np.float64)
    if len(points) != h * w:
        raise RuntimeError(
            f"Point cloud nema h*w tocaka. Slika ima {h}x{w}={h*w}, "
            f"a PCD ima {len(points)} tocaka."
        )

    colors = np.asarray(pcd.colors, dtype=np.float32)
    if colors.size == 0:
        colors = np.zeros((len(points), 3), dtype=np.float32)

    points_grid = points.reshape(h, w, 3)
    colors_grid = colors.reshape(h, w, 3)
    return points_grid, colors_grid


def make_point_cloud(points: np.ndarray, colors: np.ndarray | None = None) -> o3d.geometry.PointCloud:
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points mora biti oblika (N, 3).")

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    if colors is not None:
        colors = np.asarray(colors, dtype=np.float32)
        if colors.shape != points.shape:
            raise ValueError("colors mora imati isti oblik kao points.")
        colors = np.clip(colors, 0.0, 1.0)
        pcd.colors = o3d.utility.Vector3dVector(colors)

    return pcd


def transform_points(points: np.ndarray, T: np.ndarray) -> np.ndarray:
    points = np.asarray(points, dtype=np.float64)
    T = np.asarray(T, dtype=np.float64)

    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points mora biti oblika (N, 3).")
    if T.shape != (4, 4):
        raise ValueError("T mora biti 4x4 matrica.")

    ones = np.ones((points.shape[0], 1), dtype=np.float64)
    points_h = np.hstack([points, ones])
    transformed = (T @ points_h.T).T
    return transformed[:, :3]


def save_json(path: str | Path, data: Any) -> Path:
    path = Path(path)
    ensure_dir(path.parent)

    def convert(obj: Any):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        if isinstance(obj, (np.float32, np.float64)):
            return float(obj)
        if isinstance(obj, (np.int32, np.int64)):
            return int(obj)
        if isinstance(obj, Path):
            return str(obj)
        if isinstance(obj, dict):
            return {k: convert(v) for k, v in obj.items()}
        if isinstance(obj, list):
            return [convert(v) for v in obj]
        return obj

    with open(path, "w", encoding="utf-8") as f:
        json.dump(convert(data), f, indent=2, ensure_ascii=False)

    return path