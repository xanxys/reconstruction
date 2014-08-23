#!/bin/python2
from __future__ import print_function, division
import argparse
import cv
import cv2
import numpy as np
import scipy.linalg as la
import json
import os.path
import matplotlib.pyplot as plt


def points_on_segment(p0, p1):
    """
    Return not all, but most of integer points on line segment.
    return: [(x, y)]
    """
    dist = la.norm(p1 - p0)
    points = np.linspace(0, 1, num=dist)[:, np.newaxis] * (p1 - p0) + p0
    return list(set(map(tuple, points.astype(int))))


def process(data):
    for annotated_image in data:
        capture_path = annotated_image["path"]
        print('Processing %s' % capture_path)
        # Load depth
        depth = cv2.imread(os.path.join(capture_path, "depth.png"), cv2.CV_LOAD_IMAGE_UNCHANGED)
        if depth.shape != (annotated_image["height"], annotated_image["width"]):
            print("Unexpected depth image size, skipping")
            continue
        if depth.dtype != np.uint16:
            print("Unexpected depth type")
            continue
        depth = depth.astype(float) * 1e-3  # mm -> m
        # Load intensity
        inten = cv2.imread(os.path.join(capture_path, "intensity.png"), cv2.CV_LOAD_IMAGE_UNCHANGED)
        if inten.shape != (annotated_image["height"], annotated_image["width"]):
            print("Unexpected depth image size, skipping")
            continue
        if inten.dtype != np.uint16:
            print("Unexpected depth type")
            continue

        for line in annotated_image["lines_on_plane"]:
            p0 = np.array([line["p0"]["x"], line["p0"]["y"]])
            p1 = np.array([line["p1"]["x"], line["p1"]["y"]])
            points = points_on_segment(p0, p1)
            print("Line between %s - %s (%d samples)" % (p0, p1, len(points)))
            depths = [depth[pt[1], pt[0]] for pt in points]
            intens = [inten[pt[1], pt[0]] for pt in points]
            plt.scatter(depths, intens)
        plt.show()

if __name__ == '__main__':
    data = json.load(open('wall_annotations.json'))
    process(data)
