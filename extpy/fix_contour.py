#!/bin/python
from __future__ import print_function, division
import cairo
import numpy as np
import json
import sys
import math


def fix_contour(points=None, vis_path=None):
    points = np.array(points)
    assert(points.shape[1] == 2)

    px_per_meter = 80
    width = 1000
    height = 1000
    surf = cairo.ImageSurface(cairo.FORMAT_RGB24, width, height)
    ctx = cairo.Context(surf)
    ctx.set_source_rgb(1, 1, 1)
    ctx.paint()
    ctx.translate(width / 2, height / 2)
    ctx.scale(px_per_meter, -px_per_meter)

    # draw meter grid
    ctx.set_line_width(2 / px_per_meter)
    ctx.new_path()
    ctx.set_source_rgba(0, 0, 0, 0.1)
    for x in range(-10, 10):
        ctx.move_to(x, -10)
        ctx.rel_line_to(0, 20)
    for y in range(-10, 10):
        ctx.move_to(-10, y)
        ctx.rel_line_to(20, 0)
    ctx.stroke()

    # draw points as lines and circles
    ctx.set_line_width(0.01)
    ctx.new_path()
    ctx.set_source_rgb(0, 0, 0)
    for pt in points:
        ctx.line_to(pt[0], pt[1])
    ctx.stroke()

    ctx.new_path()
    ctx.set_source_rgb(0, 0, 0)
    for pt in points:
        ctx.new_sub_path()
        ctx.arc(pt[0], pt[1], 0.03, 0, 2 * math.pi)
    ctx.fill()

    if vis_path is not None:
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
