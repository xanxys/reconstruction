#!/bin/python
from __future__ import print_function, division
import cairo
import numpy as np
import scipy.linalg as la
import scipy.cluster.vq
import json
import sys
import math


def rotation(angle):
    """
    Returns 2d rotation matrix.
    """
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([
        [c, -s],
        [s, c]])


def best_kmeans(vs, sigma):
    """
    Do k-means and return best clustering based on AIC.
    constraint: 2 <= #cluster < 10
    Assumes vs is i.i.d and vs ~ N(sigma)
    """
    smallest = (1e10, None)
    # ln L = -ln(sigma) - 0.5*ln(2 * pi) - error ^ 2 / (2 * sigma^2)
    for n_cluster in range(2, 10):
        ticks, errors = scipy.cluster.vq.kmeans(vs, n_cluster)
        likelihood = np.sum(
            - math.log(sigma) - 0.5 * math.log(2 * math.pi)
            - errors ** 2 / (2 * sigma ** 2))
        aic = -2 * likelihood + 2 * n_cluster
        # print("AIC(K=%d): %f" % (n_cluster, aic))
        curr = (aic, ticks)
        if curr < smallest:
            smallest = curr
    assert(smallest[1] is not None)
    return smallest[1]


class ContourAnalyzer(object):
    """
    Hallucinate "good" room contour from given 2-d points,
    mainly by enforcing Manhattan assumption.

    A good contour lies mostly on given points,
    but simplicity is more important than numerical precision.

    Error should be handled as features (a la CAD).
    """
    def __init__(self, points=None, vis_path=None):
        self.points = np.array(points)
        assert(self.points.shape[1] == 2)

        if vis_path is None:
            self.vis_path = None
        else:
            self.vis_path = str(vis_path)

        self.calc_normals()
        self.choose_principle()
        self.compress()

    def compress(self):
        """
        Create lowest entropy description of points
        without too much discrepancy.

        Good measure is DoF of real-valued variables.

        We're assuming target indoor is of a single room.
        Thus, we can discretize each principle axis separately.

        ticks
        -> segments
        -> segments + features
        """
        # Choose "ticks" of each axis.
        # Almost everything will be quantized afterwards.
        sigma_dist = 0.05
        cos_thresh = math.cos(10 / 180 * math.pi)
        vs0 = []
        vs1 = []
        for (p, n) in self.get_surfels():
            if abs(np.dot(self.prin0, n)) >= cos_thresh:
                vs0.append(np.dot(self.prin0, p))
            elif abs(np.dot(self.prin1, n)) >= cos_thresh:
                vs1.append(np.dot(self.prin1, p))
        self.ticks0 = best_kmeans(np.array(vs0), sigma_dist)
        self.ticks1 = best_kmeans(np.array(vs1), sigma_dist)

        # snap to irregular grid formed by ticks
        snap_thresh = sigma_dist * 2
        flags_snapped = []
        points_snapped = []
        for p_w in self.points:
            snapped = False
            p_prin = np.dot(self.world_to_prin, p_w)
            x_snap = min(self.ticks0, key=lambda x: abs(x - p_prin[0]))
            if abs(p_prin[0] - x_snap) < snap_thresh:
                p_prin[0] = x_snap
                snapped = True
            y_snap = min(self.ticks1, key=lambda x: abs(x - p_prin[1]))
            if abs(p_prin[1] - y_snap) < snap_thresh:
                p_prin[1] = y_snap
                snapped = True
            p_w_snap = np.dot(self.prin_to_world, p_prin)
            points_snapped.append(p_w_snap)
            flags_snapped.append(snapped)
        assert(len(flags_snapped) == len(self.points))
        assert(len(points_snapped) == len(self.points))

        # When vertices snap to both X and Y axes,
        # they can end up exactly same to each other.
        # Also, input data itself can contain two very clost
        # points.
        # We remove it by set.
        thresh = 0.05
        new_points = []
        new_flags = []
        for (pt, flag) in zip(points_snapped, flags_snapped):
            # is there a close point?
            if any(la.norm(pt - p) < thresh for p in new_points):
                continue
            else:
                new_points.append(pt)
                new_flags.append(flag)
        flags_snapped = new_flags
        points_snapped = new_points

        # After snapping, most (90+%) edges must form continous
        # segments, and only occasionaly have several
        # unsnapped vertices between segments.
        # Othrewise, we can assume that given points are
        # terribly broken, or room is non-Manhattan.
        if np.sum(flags_snapped) / len(self.points) < 0.8:
            raise RuntimeError(
                "Too few snapped vertices; contour is broken or non-Manhattan")

        # cycle points to align a segment boundary to head.
        # boundary vertex: perpendicular segments or
        # (only) one of neighbor is a non-snapped.
        def is_seam_vertex(i):
            n = len(flags_snapped)
            i %= n
            if not flags_snapped[i]:
                return False
            pc = points_snapped[i]
            pp = points_snapped[i - 1]
            pn = points_snapped[(i + 1) % n]
            fp = flags_snapped[i - 1]
            fn = flags_snapped[(i + 1) % n]

            enext = pn - pc
            eprev = pc - pp
            enext /= la.norm(enext)
            eprev /= la.norm(eprev)

            if abs(np.dot(enext, eprev)) < 1e-6:
                # orthogonal segments
                return True
            elif (fp or fn) and not (fp and fn):
                # segment - residue
                return i
            elif not (fp or fn):
                # return False
                # non-orthogonal segment-segment
                raise RuntimeError(
                    "Unknown type of segment-segment boundary")
            else:
                assert(fp and fn)
                if np.dot(enext, eprev) >= 0.99:
                    # middle of a residual edge
                    return False
                else:
                    # single non-snapped edge
                    # e.g.
                    # |
                    # \___
                    return True


        def find_segment_seam():
            """
            Return seam index.
            """
            for i in range(len(points_snapped)):
                if is_seam_vertex(i):
                    return i
            raise RuntimeError(
                "Segment seam not found;  topology too strange to continue")

        ix0 = find_segment_seam()

        # create features to describe non-snapped points
        # feature is expansion of single (virtual) vertex
        # two types of features:
        # 1. betwen orthogonal segments
        #  |        |
        #  |         \
        #  .----      \____
        # 2. between parallel segments
        #                   _____
        # _____.____   _____|   |_____
        pts = []
        for ix in range(ix0, ix0 + len(points_snapped)):
            ix %= len(points_snapped)
            if is_seam_vertex(ix):
                pts.append(points_snapped[ix])
            else:
                if not flags_snapped[ix]:
                    pts.append(points_snapped[ix])
        self.points = pts

        # TODO: implement features
#        self.features = []

    def calc_normals(self):
        edge_to_normal = rotation(math.pi / 2)
        self.normals = []
        for (i, p0) in enumerate(self.points):
            p1 = self.points[(i + 1) % len(self.points)]
            n = np.dot(edge_to_normal, p1 - p0)
            n /= la.norm(n)
            self.normals.append(n)
        assert(np.isfinite(self.normals).all())

    def get_surfels(self):
        """
        Return [(edge midpoint, edge normal)]
        """
        edge_to_normal = rotation(math.pi / 2)
        ens = []
        for (i, p0) in enumerate(self.points):
            p1 = self.points[(i + 1) % len(self.points)]
            n = np.dot(edge_to_normal, p1 - p0)
            n /= la.norm(n)
            ens.append(((p0 + p1) / 2, n))
        return ens

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
        # create basis converters
        self.world_to_prin = np.array([self.prin0, self.prin1])
        self.prin_to_world = self.world_to_prin.T

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

        self.draw_ticks(ctx)

        if self.vis_path is not None:
            surf.write_to_png(str(self.vis_path))

        return {
            "vp": self.vis_path,
            "points": [[float(e) for e in row] for row in self.points]
        }

    def draw_ticks(self, ctx):
        ctx.set_line_width(0.02)
        ctx.new_path()
        ctx.set_source_rgba(1, 0, 0, 0.3)
        for v0 in self.ticks0:
            p0 = self.prin0 * v0 - self.prin1 * 10
            p1 = self.prin0 * v0 + self.prin1 * 10
            ctx.move_to(*p0)
            ctx.line_to(*p1)
        ctx.stroke()
        ctx.new_path()
        ctx.set_source_rgba(0, 1, 0, 0.3)
        for v1 in self.ticks1:
            p0 = self.prin1 * v1 - self.prin0 * 10
            p1 = self.prin1 * v1 + self.prin0 * 10
            ctx.move_to(*p0)
            ctx.line_to(*p1)
        ctx.stroke()

    def draw_contour(self, ctx):
        # edges
        cos_thresh = math.cos(10 / 180 * math.pi)
        ctx.set_line_width(0.04)
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
            ctx.arc(pt[0], pt[1], 0.05, 0, 2 * math.pi)
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
