#!/bin/python
from __future__ import print_function, division
import cairo
import numpy as np
import scipy.linalg as la
import json
import sys
import math
import itertools


def rotation(angle):
    """
    Returns 2d rotation matrix.
    """
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([
        [c, -s],
        [s, c]])


class ContourAnalyzer(object):
    def __init__(self, points=None, vis_path=None):
        self.points = np.array(points)
        assert(self.points.shape[1] == 2)

        if vis_path is None:
            self.vis_path = None
        else:
            self.vis_path = str(vis_path)

        self.calc_normals()
        self.choose_principle()

    def calc_normals(self):
        edge_to_normal = rotation(math.pi / 2)
        self.normals = []
        for (i, p0) in enumerate(self.points):
            p1 = self.points[(i + 1) % len(self.points)]
            n = np.dot(edge_to_normal, p1 - p0)
            n /= la.norm(n)
            self.normals.append(n)
        assert(np.isfinite(self.normals).all())

    def choose_principle(self):
        """
        Choose principle axis among normals.
        """
        cos_thresh = math.cos(15 / 180 * math.pi)

        def similarity(n):
            cs = np.dot(self.normals, n)
            cs[cs < cos_thresh] = 0
            return np.sum(cs)
        self.prin0 = max(self.normals, key=similarity)
        self.prin1 = np.dot(rotation(math.pi / 2), self.prin0)
        assert(abs(np.dot(self.prin0, self.prin1)) < 1e-6)

    def get_result(self):
        px_per_meter = 80
        width = 1000
        height = 1000
        surf = cairo.ImageSurface(cairo.FORMAT_RGB24, width, height)
        ctx = cairo.Context(surf)
        ctx.translate(width / 2, height / 2)
        ctx.scale(px_per_meter, -px_per_meter)

        self.draw_grid_background(ctx, px_per_meter)

        ctx.save()
        ctx.translate(5, 5)
        self.draw_normals(ctx)
        ctx.restore()

        self.draw_contour(ctx)

        if self.vis_path is not None:
            surf.write_to_png(str(self.vis_path))

        return {
            "vp": self.vis_path
        }

    def draw_contour(self, ctx):
        # edges
        cos_thresh = math.cos(10 / 180 * math.pi)
        ctx.set_line_width(0.01)
        for (i, p0) in enumerate(self.points):
            p1 = self.points[(i + 1) % len(self.points)]
            ctx.new_path()

            normal = np.dot(rotation(math.pi / 2), p1 - p0)
            normal /= la.norm(normal)

            if abs(np.dot(normal, self.prin0)) >= cos_thresh:
                ctx.set_source_rgb(1, 0, 0)
            elif abs(np.dot(normal, self.prin1)) >= cos_thresh:
                ctx.set_source_rgb(0, 1, 0)
            else:
                ctx.set_source_rgb(0, 0, 0)
            ctx.move_to(*p0)
            ctx.line_to(*p1)
            ctx.stroke()
        # vertices
        ctx.new_path()
        ctx.set_source_rgb(0, 0, 0)
        for pt in self.points:
            ctx.new_sub_path()
            ctx.arc(pt[0], pt[1], 0.03, 0, 2 * math.pi)
        ctx.fill()

    def draw_grid_background(self, ctx, px_per_meter):
        """
        Draw 1 meter grid with background.
        """
        ctx.set_source_rgb(1, 1, 1)
        ctx.paint()
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

    def draw_normals(self, ctx):
        """
        Draw normals and principle axis
        in [-1, 1] region.
        """
        ctx.set_line_width(0.01)
        ctx.arc(0, 0, 1, 0, 2 * math.pi)
        ctx.set_source_rgb(1, 1, 1)
        ctx.fill_preserve()
        ctx.set_source_rgb(0, 0, 0)
        ctx.stroke()

        ctx.new_path()
        ctx.set_source_rgba(0, 0, 0, 0.2)
        for n in self.normals:
            ctx.move_to(0, 0)
            ctx.line_to(n[0], n[1])
            # draw everytime to darken overlapped lines
            ctx.stroke()
        ctx.new_path()
        ctx.set_source_rgb(1, 0, 0)
        ctx.move_to(0, 0)
        ctx.line_to(*self.prin0)
        ctx.stroke()

        ctx.set_source_rgb(0, 1, 0)
        ctx.move_to(0, 0)
        ctx.line_to(*self.prin1)
        ctx.stroke()


# input: {
#   "points": [(Float, Float)]
#   "vis_path": string
# }
#  2D vertices in CCW, that represents a polygon.
# output: [(Float, Float, Float)]
#  color multipliers
if __name__ == '__main__':
    in_obj = json.load(sys.stdin)
    out_obj = ContourAnalyzer(**in_obj).get_result()
    json.dump(out_obj, sys.stdout)
