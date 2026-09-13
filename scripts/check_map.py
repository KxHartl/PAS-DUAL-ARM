#!/usr/bin/env python3
"""Reject a saved three-room map that only contains the HOME room."""

import argparse
import os
import sys

import cv2
import numpy as np
import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('yaml_file')
    args = parser.parse_args()
    with open(args.yaml_file, encoding='utf-8') as stream:
        metadata = yaml.safe_load(stream)
    image = cv2.imread(os.path.join(os.path.dirname(args.yaml_file), metadata['image']),
                       cv2.IMREAD_GRAYSCALE)
    if image is None:
        parser.error('occupancy image is missing')
    free = image >= 250
    rows, cols = np.where(free)
    if len(rows) == 0:
        parser.error('map has no free cells')
    resolution = float(metadata['resolution'])
    span_x = (cols.max() - cols.min()) * resolution
    span_y = (rows.max() - rows.min()) * resolution
    free_area = len(rows) * resolution * resolution
    print(f'free area {free_area:.1f} m²; observed span {span_x:.1f} × {span_y:.1f} m')
    # The three 6×6 m rooms form a 12×12 m L; mapping only HOME gives ~6×6 m.
    if span_x < 9.0 or span_y < 9.0 or free_area < 55.0:
        print('REJECTED: incomplete three-room coverage', file=sys.stderr)
        return 1
    print('Coverage gate passed; inspect wall alignment and door openings in RViz.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
