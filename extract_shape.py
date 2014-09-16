#!/bin/python2
from __future__ import print_function, division
import cairo
import numpy as np
import math


def load_cloud(path):
    """
    Return XYZRGBNormal (9, N) cloud
    Only works for certain data!

    Don't use after this experiment.
    """
    lines = list(open(path))
    n_vertex = int(lines[2].split()[2])
    vs = []
    for vert in lines[15:15+n_vertex]:
        vs.append(map(float, vert.split()))
    return np.array(vs)


if __name__ == '__main__':
    cloud = load_cloud('scan-20140827-12:57-gakusei-small/debug_points_interior_slice.ply')
    print("Cloud shape: %s" % str(cloud.shape))

    xyzs = cloud[:, :3]

    # extract AABB and setup drawing region
    px_per_meter = 100
    x0 = xyzs[:, 0].min() - 1
    x1 = xyzs[:, 0].max() + 1
    y0 = xyzs[:, 1].min() - 1
    y1 = xyzs[:, 1].max() + 1

    width = int((x1 - x0) * px_per_meter)
    height = int((y1 - y0) * px_per_meter)

    surf = cairo.ImageSurface(cairo.FORMAT_RGB24, width, height)
    ctx = cairo.Context(surf)
    ctx.scale(px_per_meter, px_per_meter)
    ctx.translate(-x0, -y0)
    # set bg to white
    ctx.set_source_rgb(1, 1, 1)
    ctx.paint()

    # draw points
    ctx.set_source_rgba(0, 0, 1, 0.1)
    for (x, y, z) in xyzs:
        ctx.arc(x, y, 0.01, 0, 2 * math.pi)
        ctx.fill()

    surf.write_to_png('shapes.png')
