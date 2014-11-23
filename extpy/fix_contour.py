#!/bin/python
from __future__ import print_function, division
import cairo
import numpy as np
import json
import sys


def fix_contour(points=None, vis_path=None):
    points = np.array(points)
    assert(points.shape[1] == 2)

    px_per_meter = 100
    width = 1000
    height = 1000
    surf = cairo.ImageSurface(cairo.FORMAT_RGB24, width, height)
    ctx = cairo.Context(surf)
    ctx.scale(px_per_meter, -px_per_meter)

    if vis_path:
        surf.write_to_png(str(vis_path))

    return {
        "vp": vis_path
    }


# input: {
#   "points": [(Float, Float)]
#   "vis_path": string
# }
#  2D vertices in CCW, that represents a polygon.
# output: [(Float, Float, Float)]
#  color multipliers
if __name__ == '__main__':
    in_obj = json.load(sys.stdin)
    out_obj = fix_contour(**in_obj)
    json.dump(out_obj, sys.stdout)
