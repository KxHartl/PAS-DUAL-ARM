"""Debug helper: visualize per-view transformed pointclouds saved by reconstruct_scene.

Usage:
    python src/debug_visualize_transformed_views.py [--dir PATH] [--no-show]

If --dir is not provided, the script searches `cfg.paths.output_dir` for the
most-recent folder containing files named `view_*_transformed.pcd`.
"""
from pathlib import Path
import argparse
import numpy as np
import open3d as o3d

from config import cfg


def find_transformed_dir(base_output: Path) -> Path | None:
    matches = list(base_output.rglob("view_*_transformed.pcd"))
    if not matches:
        return None
    latest = max(matches, key=lambda p: p.stat().st_mtime)
    return latest.parent


def load_and_color(trans_dir: Path):
    files = sorted(trans_dir.glob("view_*_transformed.pcd"))
    if not files:
        raise FileNotFoundError("No view_*_transformed.pcd files in dir")

    base_colors = [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [1.0, 1.0, 0.0],
        [1.0, 0.0, 1.0],
        [0.0, 1.0, 1.0],
    ]

    geometries = []
    centroids = []

    for i, f in enumerate(files):
        pcd = o3d.io.read_point_cloud(str(f))
        color = base_colors[i % len(base_colors)]
        if len(pcd.points) > 0:
            pcd.paint_uniform_color(color)
            pts = np.asarray(pcd.points)
            cent = pts.mean(axis=0)
        else:
            cent = np.array([0.0, 0.0, 0.0])

        geometries.append(pcd)

        # small sphere for centroid
        sph = o3d.geometry.TriangleMesh.create_sphere(radius=0.005)
        sph.compute_vertex_normals()
        sph.paint_uniform_color([0.9, 0.1, 0.1])
        sph.translate(cent)
        geometries.append(sph)

        centroids.append((f.name, cent.tolist()))

    # global coordinate frame at origin for reference
    geometries.append(o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05))

    return files, geometries, centroids


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dir", type=str, help="Path to folder with view_*_transformed.pcd")
    parser.add_argument("--no-show", action="store_true", help="Do not open Open3D viewer; only print info")
    args = parser.parse_args()

    base_output = Path(cfg.paths.output_dir)

    if args.dir:
        trans_dir = Path(args.dir)
    else:
        trans_dir = find_transformed_dir(base_output)

    if trans_dir is None or not trans_dir.exists():
        print("Could not find any view_*_transformed.pcd under:", base_output)
        raise SystemExit(1)

    print(f"Using transformed view directory: {trans_dir}")
    files, geometries, centroids = load_and_color(trans_dir)

    print("Found files:")
    for f in files:
        print(" -", f)

    print("Centroids:")
    for name, c in centroids:
        print(f" - {name}: ({c[0]:.4f}, {c[1]:.4f}, {c[2]:.4f})")

    if not args.no_show:
        title = f"Transformed views: {trans_dir.name}"
        o3d.visualization.draw_geometries(geometries, window_name=title, width=1200, height=800)


if __name__ == "__main__":
    main()
