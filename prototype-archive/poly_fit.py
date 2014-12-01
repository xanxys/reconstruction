#!/bin/python3
import cairo
import cmath
import itertools
import json
import math
import numpy as np
import numpy.linalg as la
import random
import scipy.spatial
import scipy.optimize


def seg_intersect(a1, a2, b1, b2):
    """
    Return True or False
    """
    da = a2-a1
    db = b2-b1

    # solve (ta, tb) in a1 + ta * da = b1 + tb * db
    # (da, -db) (ta, tb)^T = b1 - a1
    m = np.array([da, -db]).T
    v = b1 - a1
    try:
        ts = la.solve(m, v)
        return (ts > 0).all() and (ts < 1).all()
    except la.LinAlgError:
        # lines are parallel or some other degenerate case
        return False


def gen_dataset(bias_perp=0):
    """
    Returns PolygonAndPoints
    """
    pt0 = np.array([0.1, 0.1])
    pt1 = np.array([0.3, 0.5])
    rot = np.array([[0, -1], [1, 0]])
    normal = np.dot(rot, pt1 - pt0)
    normal /= la.norm(normal)

    sigma = 0.01
    n_points = 100
    points = []
    for i in range(n_points):
        pt_center = pt0 + (pt1 - pt0) * i / n_points
        points.append(pt_center + np.random.randn(2) * sigma + normal * bias_perp)

    return PolygonAndPoints(
        np.array([pt0, pt1]),
        np.array(points))


def proj_to_segment(p0, p1, query):
    dp = p1 - p0
    dp_norm = dp / la.norm(dp)
    return np.dot(query - p0, dp_norm) / la.norm(dp)


def eval_param(ds, param):
    """
    ds: PolygonAndPoints
    param: delta pos: shape=(#vert, 3), flattened

    return: (cost, dcost/dparam)
    """
    assert(ds.vertices.size == param.size)
    cost = 0
    n_reachable = 0
    poly_adjusted = ds.vertices + param.reshape(ds.vertices.shape)

    # point cost
    for p in ds.points:
        t = proj_to_segment(poly_adjusted[0], poly_adjusted[1], p)
        if not (0 <= t <= 1):
            continue
        n_reachable += 1
        p_proj = t * (poly_adjusted[1] - poly_adjusted[0]) + poly_adjusted[0]
        dist = la.norm(p - p_proj)
        cost += dist ** 2

    # param regularizer
    for p in param:
        cost += la.norm(p)

    # print("reachable points: %d / %d" % (n_reachable, len(param)))
    return (cost, param * 0)


class PolygonAndPoints(object):
    def __init__(self, vertices, points):
        self.vertices = vertices
        self.points = points

    def get_param_dim(self):
        return self.vertices.size

    def get_adjusted_vertices(self, param):
        assert(param.size == self.get_param_dim())
        return self.vertices + param.reshape(self.vertices.shape)


def draw_dataset(ctx, ds, params=None):
    """
    Draw dataset and an optional parameter covergence trajectory.
    """
    # points
    ctx.new_path()
    for pt in ds.points:
        ctx.new_sub_path()
        ctx.arc(pt[0], pt[1], 0.002, 0, 2 * math.pi)
    ctx.set_source_rgba(0, 0, 0, 0.5)
    ctx.fill()

    # poly
    ctx.set_line_width(0.001)
    ctx.new_path()
    for pt in ds.vertices:
        ctx.line_to(*pt)
    ctx.set_source_rgb(0, 0, 0)
    ctx.stroke()

    if params is not None:
        ctx.set_line_width(0.001)
        for (i, param) in enumerate(params):
            t = i / len(params)
            ctx.new_path()
            for pt in ds.get_adjusted_vertices(param):
                ctx.line_to(*pt)
            ctx.set_source_rgba(1 - t, t, 0, 0.1)
            ctx.stroke()


def test_poly_fitting():
    # Setup a context that maps [0,1]^2 region to an image
    size = 800
    surf = cairo.ImageSurface(cairo.FORMAT_RGB24, size, size)
    ctx = cairo.Context(surf)
    ctx.scale(size, size)
    ctx.translate(0, 1)
    ctx.scale(1, -1)
    ctx.set_source_rgb(1, 1, 1)
    ctx.paint()
    ctx.set_line_width(0.001)

    for i in range(10):
        print("========================================")
        bias = i * 0.01
        ds = gen_dataset(bias)
        param = np.zeros([ds.get_param_dim()])
        print("perp bias: %f / cost: %f" % (bias, eval_param(ds, param)[0]))

        params = []
        def record_param(param):
            params.append(param)

        result = scipy.optimize.minimize(
            lambda x: eval_param(ds, x)[0],
            param,
            method='Nelder-Mead',
            options={
                "maxiter": 200
            },
            callback=record_param)
        print(result)

        if i == 9:
            draw_dataset(ctx, ds, params)
            surf.write_to_png("poly_fit.png")


if __name__ == '__main__':
    test_poly_fitting()
