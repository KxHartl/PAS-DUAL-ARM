#!/usr/bin/env python3
"""Generate Nav2 Keepout Filter mask (PGM + YAML) matching seminar_map.

Creates:
  1. Virtual guide walls (chutes) at Blue and Red doorways to strictly
     constrain global planning to orthogonal entry/exit along the centerline.
  2. Safety keepout boxes around the tables in BLUE and RED rooms.
  3. Leaves the base SLAM map completely untouched and pure.
"""

import os
import sys
import numpy as np
import yaml
from PIL import Image


def main():
    map_yaml_path = os.path.join(
        os.path.dirname(__file__), "..", "src", "pas_dual_arm_bringup", "maps", "seminar_map.yaml"
    )
    map_yaml_path = os.path.abspath(map_yaml_path)

    with open(map_yaml_path, "r", encoding="utf-8") as f:
        meta = yaml.safe_load(f)

    resolution = float(meta["resolution"])
    origin_x = float(meta["origin"][0])
    origin_y = float(meta["origin"][1])

    # Load base map to match exact dimensions
    base_pgm = os.path.join(os.path.dirname(map_yaml_path), meta["image"])
    im_base = Image.open(base_pgm)
    width, height = im_base.size

    # Keepout mask convention:
    # 254 (white) = free space (cost = 0, no keepout)
    # 0 (black)   = lethal keepout obstacle (cost = 100 / 254)
    mask = np.full((height, width), 254, dtype=np.uint8)

    def fill_world_rect(x_min, x_max, y_min, y_max, value=0):
        c_min = int(round((x_min - origin_x) / resolution))
        c_max = int(round((x_max - origin_x) / resolution))
        r_max = height - 1 - int(round((y_min - origin_y) / resolution))
        r_min = height - 1 - int(round((y_max - origin_y) / resolution))
        c0 = max(0, min(c_min, c_max))
        c1 = min(width, max(c_min, c_max) + 1)
        r0 = max(0, min(r_min, r_max))
        r1 = min(height, max(r_min, r_max) + 1)
        mask[r0:r1, c0:c1] = value

    # 1. Doorway guide walls (corridors extending 1.2 m each side):
    # Blue doorway (at Y = -3.0, opening X in [-0.5, 0.5]):
    fill_world_rect(-1.20, -0.55, -4.20, -1.80, value=0)
    fill_world_rect(0.55, 1.20, -4.20, -1.80, value=0)

    # Red doorway (at X = 3.0, opening Y in [-0.5, 0.5]):
    fill_world_rect(1.80, 4.20, 0.55, 1.20, value=0)
    fill_world_rect(1.80, 4.20, -1.20, -0.55, value=0)

    # 2. Table safety zones (1.3 m x 1.3 m keepout boxes):
    # Blue table at (0.0, -6.5)
    fill_world_rect(-0.65, 0.65, -7.15, -5.85, value=0)
    # Red table at (6.5, 0.0)
    fill_world_rect(5.85, 7.15, -0.65, 0.65, value=0)

    out_dir = os.path.join(os.path.dirname(__file__), "..", "src", "pas_dual_arm_bringup", "maps")
    out_pgm = os.path.join(out_dir, "keepout_mask.pgm")
    out_yaml = os.path.join(out_dir, "keepout_mask.yaml")

    Image.fromarray(mask).save(out_pgm)
    print(f"Saved keepout mask PGM to: {out_pgm}")

    mask_meta = {
        "image": "keepout_mask.pgm",
        "mode": "trinary",
        "resolution": resolution,
        "origin": [origin_x, origin_y, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
    }
    with open(out_yaml, "w", encoding="utf-8") as f:
        yaml.dump(mask_meta, f, default_flow_style=False)
    print(f"Saved keepout mask YAML to: {out_yaml}")


if __name__ == "__main__":
    main()
