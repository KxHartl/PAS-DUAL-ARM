#!/usr/bin/env python3
"""Generate a DICT_4X4_50 marker PNG with the quiet zone the world models assume.

The seminar world puts a marker on a plate and tells the detector how big the
marker proper is. The PNG therefore has to carry the white border itself: with
the default 12.5% per side the marker fills 75% of the plate, which is the
convention markers 0-2 already use (a 0.22 m plate -> 0.165 m marker).

    python3 scripts/make_marker.py 3 --px 800 \
        --out src/pas_dual_arm_bringup/worlds/materials/textures/aruco_marker_3.png
"""
import argparse

import cv2
import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('marker_id', type=int)
    ap.add_argument('--out', required=True)
    ap.add_argument('--px', type=int, default=800, help='full plate size in pixels')
    ap.add_argument('--quiet-zone', type=float, default=0.125,
                    help='white border per side, as a fraction of the plate')
    args = ap.parse_args()

    border = int(round(args.px * args.quiet_zone))
    inner = args.px - 2 * border
    if inner <= 0:
        raise SystemExit('quiet zone leaves no room for the marker')

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker = cv2.aruco.generateImageMarker(dictionary, args.marker_id, inner)
    plate = np.full((args.px, args.px), 255, dtype=np.uint8)
    plate[border:border + inner, border:border + inner] = marker
    if not cv2.imwrite(args.out, plate):
        raise SystemExit(f'could not write {args.out}')
    print(f'{args.out}: {args.px}px plate, marker {inner}px '
          f'({inner / args.px:.0%} of the plate), id {args.marker_id}')


if __name__ == '__main__':
    main()
