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


def calc_distance(path):
    """
    Calculate total distance of adjacent points,
    treating path as circular.
    e.g. calc_distance([v0,v1,v2]) = |v1-v0| + |v2-v1| + |v0-v2|
    """

    distance = sum(la.norm(p_next - p) for (p, p_next) in zip(path, path[1:]))
    distance += la.norm(path[-1] - path[0])
    return distance


def solve_tsp(points):
    """
    Get exact solution of planar traveling salesman problem.
    Only usable for several points
    return (total ditance, points (without last one))
    """
    assert(len(points) >= 2)
    return min(
        ((calc_distance(path), path) for path
            in itertools.permutations(points)),
        key=lambda pair: pair[0])


def random_swap(ls):
    ls = list(ls)  # copy
    n = len(ls)
    # Sample two different indices
    i = random.randint(0, n - 2)
    j = random.randint(i + 1, n - 1)
    ls[i], ls[j] = ls[j], ls[i]
    return ls


def approx_tsp(points, num_iteration=100):
    """ Solve traveling salesman. """
    assert(len(points) >= 2)
    path = list(points)
    temperature = calc_distance(path)

    for i in range(num_iteration):
        new_path = random_swap(path)
        delta = calc_distance(new_path) - calc_distance(path)
        # Always accept when delta < 0. Otherwise vary depending on
        # how worse new_path than (current) path is.
        if math.exp(- delta / temperature) > random.random():
            path = new_path
            temperature *= 0.9
    return (calc_distance(path), path)


def compare_tsp_approx_vs_exact():
    for n in range(2, 10):
        print("## Planar TSP with N = %d" % n)
        pts = np.random.rand(n, 2)
        print('Approx:', approx_tsp(pts))
        print('Exact:', solve_tsp(pts))


def generate_overlapping_rects(n):
    """
    Return rectangles whose union is connected.
    return: [(p0, size)]
    """
    def overlap1d(r0, r1):
        return not ((r0[1] < r1[0]) or (r0[0] > r1[1]))

    def overlap2d(r0, r1):
        """
        r0,r1: (vmin, vmax)
        """
        p00 = r0[0]
        p01 = r0[0] + r0[1]
        p10 = r1[0]
        p11 = r1[0] + r1[1]
        return all(overlap1d(
            (p00[axis], p01[axis]),
            (p10[axis], p11[axis])) for axis in range(2))

    rects = []
    while len(rects) < n:
        r = (np.random.rand(2) * 2 - 1.5, 1.5 * np.random.rand(2) + 0.1)
        if len(rects) == 0 or any(overlap2d(r_some, r) for r_some in rects):
            rects.append(r)
    return rects


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


def concave_hull(points, k_min=5):
    """
    points: 2d points in shape [N, 2]
    return: CCW list of points

    Implementation of k-nearest neighbor based approach
    http://repositorium.sdum.uminho.pt/xmlui/bitstream/handle/1822/6429/ConcaveHull_ACM_MYS.pdf?sequence=1
    """
    kdt = scipy.spatial.KDTree(points)

    # Select the point with minimum Y.
    start_ix = np.argmin(points, axis=0)[1]
    start = points[start_ix]
    prev_angle = -1

    circle = [start]
    circle_s = set([int(start_ix)])
    while len(circle) <= 1 or (circle[-1] != start).any():
        current = circle[-1]

        def intersect2d(s0, s1):
            return seg_intersect(s0[0], s0[1], s1[0], s1[1])

        def nonintersecting(i):
            new_segment = (current, points[i])
            # [1:-1]: exclude prev-current segment, because
            # 1. it's guaranteed not intersect (they share a single point)
            # 2. sometimes lead to false intersection when included
            return not any(intersect2d(segment, new_segment) for segment
                in zip(circle, circle[1:-1]))

        k = k_min
        while True:
            # Query k + 1 and drop the first (= query itself) point.
            ixs = list(kdt.query(current, k=k + 1)[1][1:])

            # Prefer the point with largest CW rotation.
            def calc_angle(i):
                """ Return CCW angle from prev_angle """
                next_angle = complex(*(points[i] - current))
                delta_angle = cmath.phase(next_angle / prev_angle)
                # If we use 0 here, there'll be almost degenerate case
                # of turning back to previous point
                if delta_angle > math.pi * 0.1:
                    delta_angle -= 2 * math.pi
            #   print("cand: %03i %.2f %s" % (i, delta_angle, points[i]))
                return delta_angle
            ixs.sort(key=calc_angle)

            # Reject already added points (start is ok, though)
            ixs = list(filter(lambda ix: ix == start_ix or (ix not in circle_s), ixs))

            # Reject self-intersecting points.
            ixs = list(filter(nonintersecting, ixs))

            # retry with larger k if there's no valid candidate
            if len(ixs) == 0:
                k = min(1 + int(k * 1.5), len(points))
                if k == len(points):
                    raise 'Circle search exhausted; probably some bug'
            else:
                break

        best_ix = ixs[0]

        # Circle detected = (successfully) returned to the start
        if best_ix in circle_s:
            break

        # Prepare for the next jump
        prev_angle = complex(*(current - points[best_ix]))
        circle.append(points[best_ix])
        circle_s.add(best_ix)

    return circle


def downsample_grid(points, resolution):
    """
    Leave only a single point by averaging all points in each tile.
    So returned number of points = area covered by points / resolution^2.
    """
    assert(resolution > 0)
    # Bundle points in each tile.
    tiles = {}
    for (pt_org, pt_int) in zip(points, np.floor(points / resolution)):
        tiles.setdefault(tuple(pt_int), []).append(pt_org)
    # Take average
    return np.array([np.mean(pts, axis=0) for pts in tiles.values()])


def test_concave_wrapping():
    rects = generate_overlapping_rects(3)

    # Setup a context that maps [-2,2]^2 region to an image
    size = 1000
    surf = cairo.ImageSurface(cairo.FORMAT_RGB24, size, size)
    ctx = cairo.Context(surf)
    ctx.scale(size / 4, size / 4)
    ctx.translate(2, 2)
    ctx.scale(1, -1)
    ctx.set_source_rgb(1, 1, 1)
    ctx.paint()
    ctx.set_line_width(0.01)

    # visualize rectangles
    ctx.set_source_rgba(0, 0, 0, 0.1)
    for r in rects:
        ctx.rectangle(r[0][0], r[0][1], r[1][0], r[1][1])
        ctx.stroke()

    # put points in rectangles
    points = np.concatenate(
        [r[0] + np.random.rand(int(100 * r[1][0] * r[1][1]), 2) * r[1] for r in rects],
        axis=0)

    # visualize points
    ctx.set_source_rgba(0, 0, 1, 0.5)
    for pt in points:
        ctx.arc(pt[0], pt[1], 0.01, 0, 2 * math.pi)
        ctx.fill()

    # now try to solve it....
    hull = concave_hull(points, 20)
    print(hull)
    ctx.move_to(*hull[0])
    for pt in hull[1:]:
        ctx.line_to(*pt)
    ctx.line_to(*hull[0])
    ctx.set_source_rgb(0, 0, 0)
    ctx.set_line_width(0.005)
    ctx.stroke()

    surf.write_to_png("sa_wrap.png")


def test_concave_for_real_data():
    # Load points
    pts_json = json.load(open('ocha_points.json', 'r'))
    points = np.array([[pt["x"], pt["z"]] for pt in pts_json])
    print('Shape before downsampling', points.shape)

    # Subsample points.
    points = downsample_grid(points, 0.1)
    print('Shape after downsampling', points.shape)

    # Setup a context that maps [-10,10]^2 region to an image
    size = 1000
    surf = cairo.ImageSurface(cairo.FORMAT_RGB24, size, size)
    ctx = cairo.Context(surf)
    ctx.scale(size / 20, size / 20)
    ctx.translate(10, 10)
    ctx.scale(1, -1)
    ctx.set_source_rgb(1, 1, 1)
    ctx.paint()

    # visualize points
    ctx.set_source_rgba(0, 0, 1, 0.5)
    for pt in points:
        ctx.arc(pt[0], pt[1], 0.05, 0, 2 * math.pi)
        ctx.fill()

    # now try to solve it....
    hull = concave_hull(points, 20)
    print(hull)
    ctx.move_to(*hull[0])
    for pt in hull[1:]:
        ctx.line_to(*pt)
    ctx.line_to(*hull[0])
    ctx.set_source_rgb(0, 0, 0)
    ctx.set_line_width(0.025)
    ctx.stroke()

    surf.write_to_png("sa_wrap_real.png")


if __name__ == '__main__':
    # compare_tsp_approx_vs_exact()
    test_concave_wrapping()
    test_concave_for_real_data()
