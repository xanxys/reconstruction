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


def gen_dataset(bias_perp=0, distortion=0):
    """
    Returns PolygonAndPoints
    """
    vertices = np.array([
        [0.1, 0.1],
        [0.3, 0.5],
        [0.8, 0.8]])
    edges = [(0, 1), (1, 2), (2, 0)]
    rot = np.array([[0, -1], [1, 0]])

    # sample points
    sigma = 0.005
    n_points_per_edge = 100
    points = []
    for (ix0, ix1) in edges:
        pt0 = vertices[ix0]
        pt1 = vertices[ix1]
        normal = np.dot(rot, pt1 - pt0)
        normal /= la.norm(normal)
        for i in range(n_points_per_edge):
            pt_center = pt0 + (pt1 - pt0) * i / n_points_per_edge
            points.append(pt_center + np.random.randn(2) * sigma + normal * bias_perp)
    points = np.array(points)

    # apply distortion
    # x: [0, 1], x^2: [0, 1]
    points += points ** 2 * distortion

    return PolygonAndPoints( vertices, edges, points)


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
    verts_adjusted = ds.get_adjusted_vertices(param)

    def dist_to_edge(p, i0, i1):
        t = proj_to_segment(verts_adjusted[i0], verts_adjusted[i1], p)
        t = max(0, min(1, t))
        p_nearest = verts_adjusted[i0] + (verts_adjusted[i1] - verts_adjusted[i0]) * t
        return la.norm(p - p_nearest)

    def dist_to_mesh(p):
        return min(dist_to_edge(p, *edge) for edge in ds.edges)

    cost = 0
    cost += sum(dist_to_mesh(p) ** 2 for p in ds.points)
    cost += sum(la.norm(p) ** 2 for p in param)

    return (cost, param * 0)


class PolygonAndPoints(object):
    def __init__(self, vertices, edges, points):
        self.vertices = vertices
        self.edges = edges
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
    for (i0, i1) in ds.edges:
        ctx.move_to(*ds.vertices[i0])
        ctx.line_to(*ds.vertices[i1])
    ctx.set_source_rgb(0, 0, 0)
    ctx.stroke()

    if params is not None:
        ctx.set_line_width(0.001)
        for (i, param) in enumerate(params):
            vs_adj = ds.get_adjusted_vertices(param)
            t = i / len(params)
            ctx.new_path()
            for (i0, i1) in ds.edges:
                ctx.move_to(*vs_adj[i0])
                ctx.line_to(*vs_adj[i1])
            ctx.set_source_rgba(1 - t, t, 0, 0.1)
            ctx.stroke()


def draw_with(img_path, draw_fn):
    """
    img_path :: str
    draw_fn :: cairo.Context -> None
    """
    size = 800
    surf = cairo.ImageSurface(cairo.FORMAT_RGB24, size, size)
    ctx = cairo.Context(surf)
    ctx.scale(size, size)
    ctx.translate(0, 1)
    ctx.scale(1, -1)
    ctx.set_source_rgb(1, 1, 1)
    ctx.paint()
    draw_fn(ctx)
    surf.write_to_png(img_path)


def test_poly_fitting():
    for i in range(10):
        print("========================================")
        distortion = i * 0.05
        ds = gen_dataset(distortion=distortion)
        param = np.zeros([ds.get_param_dim()])
        print("distortion: %f / cost: %f" % (
            distortion, eval_param(ds, param)[0]))

        params = []

        def record_param(param):
            params.append(param)

        result = scipy.optimize.minimize(
            lambda x: eval_param(ds, x)[0],
            param,
            method='Nelder-Mead',
            options={
                "maxiter": 100
            },
            callback=record_param)
        print(result)

        def draw(ctx):
            draw_dataset(ctx, ds, params)

        draw_with("poly_fit-%02d.png" % i, draw)


if __name__ == '__main__':
    test_poly_fitting()
