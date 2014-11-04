#!/bin/python2
from __future__ import print_function, division
import argparse
import cv
import cv2
import numpy as np
import os.path

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="""
Guess rotational alignment by horizontal correlation""")
    parser.add_argument(
        'scans', metavar='SCAN', nargs='+', type=str,
        help='Scan directory paths')
    args = parser.parse_args()
    if len(args.scans) != 2:
        raise RuntimeError("Currently only pairing is supported")

    rgb0 = cv2.imread(os.path.join(args.scans[0], 'rgb.png'))
    rgb1 = cv2.imread(os.path.join(args.scans[1], 'rgb.png'))
    assert(rgb0.shape == rgb1.shape)

    rgb0 = cv2.GaussianBlur(rgb0, (0, 0), 30)
    rgb1 = cv2.GaussianBlur(rgb1, (0, 0), 30)

    def rotate(dx, img):
        rotated = np.empty_like(img)
        n_cols = img.shape[1]
        for col in range(n_cols):
            rotated[:, (col + dx) % n_cols] = img[:, col]
        return rotated

    width = rgb0.shape[1]
    pairs = []
    for angle in range(0, 360, 10):
        dx = int(width * angle / 360)
        delta = ((rgb0 - rotate(dx, rgb1)) ** 2).sum(axis=(0, 1)) / (rgb0.shape[0] * rgb0.shape[1])
        delta = delta.sum()
        print("%3.0fdeg: E=%4.1f" % (angle, delta))
        pairs.append((delta, angle))
        if angle == 90:
            cv2.imwrite("hoge.png", rotate(dx, rgb1))
    print(min(pairs))
