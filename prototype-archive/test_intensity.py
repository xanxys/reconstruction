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
import itertools

def points_on_segment(p0, p1):
    """
    Return not all, but most of integer points on line segment.
    return: [(x, y)]
    """
    dist = la.norm(p1 - p0)
    points = np.linspace(0, 1, num=dist)[:, np.newaxis] * (p1 - p0) + p0
    return list(set(map(tuple, points.astype(int))))


def process(data):
    for (i_image, annotated_image) in enumerate(data):
        if i_image != 0:
            continue
        capture_path = annotated_image["path"]
        print('Processing %s' % capture_path)
        # Load xyz
        xyz = (cv2.imread(os.path.join(capture_path, "xyz.png"), cv2.CV_LOAD_IMAGE_UNCHANGED).astype(np.float32) - 2**15) * 1e-3

        # Load normal
        normal = cv2.imread(os.path.join(capture_path, "normal_vis.png")).astype(np.float32) / 255.0 * 2.0 - 1.0

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

        # Load rgb to overlay line segments
        palette = [('r', (0, 0, 255)), ('g', (0, 255, 0)), ('b', (255, 0, 0))]
        rgb = cv2.imread(os.path.join(capture_path, "rgb.png"))
        irs = []
        for (line, (color, color_cv)) in zip(annotated_image["lines_on_plane"], itertools.cycle(palette)):
            p0 = np.array([line["p0"]["x"], line["p0"]["y"]])
            p1 = np.array([line["p1"]["x"], line["p1"]["y"]])
            cv2.line(rgb, tuple(p0), tuple(p1), color_cv, lineType=cv.CV_AA)
            points = points_on_segment(p0, p1)
            print("Line between %s - %s (%d samples)" % (p0, p1, len(points)))
            depths = np.array([depth[pt[1], pt[0]] for pt in points])
            intens = np.array([inten[pt[1], pt[0]] for pt in points])
            xyzs = np.array([xyz[pt[1], pt[0]] for pt in points])
            normals = np.array([normal[pt[1], pt[0]] for pt in points])
            normals = np.array([np.mean(normals, axis=0) for pt in points])

            inten_ratio = []
            org = np.array([0, 0, 0])
            for (p, n, d, i) in zip(xyzs, normals, depths, intens):
                cos = np.dot(org - p, n)
                est = cos / d**0.7
                inten_ratio.append(i / est)
            irs += inten_ratio


            plt.scatter(depths, inten_ratio, c=color)
        plt.xlabel('distance/m')
        plt.ylabel('real intensity / est. intensity')
        #plt.xlim(0, 0.5)
        plt.ylim(0, np.max(irs) * 1.1)
        cv2.imwrite('segments-on-rgb-%d.png' % i_image, rgb)
    plt.grid()
    plt.show()

if __name__ == '__main__':
    data = json.load(open('wall_annotations.json'))
    process(data)
