#!/bin/python2
from __future__ import print_function, division
import argparse
import cPickle
import cairo
import numpy as np
import math
import random
import matplotlib.pyplot as plt
import scipy.signal
import scipy.spatial
import itertools
import logging
import os
import os.path

logging.basicConfig(level=logging.DEBUG)


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


def serialize_points_as_ply(points, fobj):
    """
    Put XYZ or XYZRGB or XYZRGBNormal points to fobj in PLY format.

    points: [N, 3] (XYZ) or [N, 6] or [N, 9] numpy array.
    """
    n_points, n_channels = points.shape
    fobj.write("ply\n")
    fobj.write("format ascii 1.0\n")
    fobj.write("element vertex %d\n" % n_points)
    fobj.write("property float32 x\n")
    fobj.write("property float32 y\n")
    fobj.write("property float32 z\n")
    if n_channels >= 6:
        fobj.write("property uint8 red\n")
        fobj.write("property uint8 green\n")
        fobj.write("property uint8 blue\n")
    if n_channels >= 9:
        fobj.write("property float32 nx\n")
        fobj.write("property float32 ny\n")
        fobj.write("property float32 nz\n")
    fobj.write("element face %d\n" % 0)
    fobj.write("property list uint8 int32 vertex_indices\n")
    fobj.write("end_header\n")

    if n_channels >= 3:
        attrib_xyz = points[:, :3]
    if n_channels >= 6:
        attrib_rgb = points[:, 3:6].astype(int)
    if n_channels >= 9:
        attrib_normal = points[:, 6:9]

    if n_channels == 3:
        for xyz in attrib_xyz:
            fobj.write("%f %f %f\n" % tuple(xyz))
    elif n_channels == 6:
        for (xyz, rgb) in zip(attrib_xyz, attrib_rgb):
            fobj.write("%f %f %f %d %d %d\n" % (tuple(xyz) + tuple(rgb)))
    elif n_channels == 9:
        for (xyz, rgb, normal) in zip(attrib_xyz, attrib_rgb, attrib_normal):
            fobj.write("%f %f %f %d %d %d %f %f %f\n" % (tuple(xyz) + tuple(rgb) + tuple(normal)))
    else:
        raise ValueError("Unsupported point cloud array shape")


def to_4cc(keys):
    """
    Split 2D keys to connected components.
    return [set(Key)]
    """
    def neighbors(k):
        x, y = k
        return [(x + 1, y), (x, y + 1), (x - 1, y), (x, y - 1)]

    def collect(k, visited):
        visited.add(k)
        for kn in neighbors(k):
            if kn not in keys:
                continue
            if kn in visited:
                continue
            collect(kn, visited)
        return visited

    keys = set(keys)
    ccs = []
    while len(keys) > 0:
        k = keys.pop()
        cc = collect(k, set())
        keys -= cc
        ccs.append(cc)
    return ccs


def rotation2d(angle):
    return np.array([
        [math.cos(angle), -math.sin(angle)],
        [math.sin(angle),  math.cos(angle)]])


def fit_rect(pts):
    """
    pts: XYZRGBNormal
    Return OBB described by (theta, p0, sz)
    """
    if len(pts) < 100:
        return []

    vmin = pts[:, :2].min(axis=0)
    vmax = pts[:, :2].max(axis=0)
    size_max = vmax - vmin

    thresh_d = 0.02
    thresh_a = 0.1

    def filter_inside(rot, c, hs):
        """ Return points inside given OBB."""
        r = rotation2d(-rot)
        pts_ortho = np.einsum('ij, si->sj', r, pts[:, :2] - c)
        inside = np.all(np.abs(pts_ortho) < hs, axis=1)
        return pts[inside]

    def eval_rect(rot, center, halfsize):
        r = rotation2d(rot)
        # Convert 4 edges to fat rect,
        dx = np.dot(r, np.array([halfsize[0], 0]))
        dy = np.dot(r, np.array([0, halfsize[1]]))
        ex = r[:, 0]
        ey = r[:, 1]
        edges = [
            ((rot, center + dx, (thresh_d, halfsize[1])), ex),
            ((rot, center - dx, (thresh_d, halfsize[1])), -ex),
            ((rot, center + dy, (halfsize[0], thresh_d)), ey),
            ((rot, center - dy, (halfsize[0], thresh_d)), -ey)
        ]
        # choose points in each of them.
        n_inlier = 0
        for (fat_edge, n) in edges:
            pts = filter_inside(*fat_edge)
            # n[:2] `dot` n' = n `dot` (n', 0)
            ns = pts[:, 6:8]
            #print(np.einsum('si, i->s', ns, n))
            n_edge = np.sum(np.einsum('si, i->s', ns, n)) # > math.cos(thresh_a))
            #n_edge = len(ns)
            n_inlier += n_edge
        return n_inlier / (np.sum(halfsize) ** 0.5)

    def random_rect():
        return (
            rot_max,  #np.random.rand(1) * math.pi / 2,
            vmin + np.random.rand(2) * size_max / 2,
            np.random.rand(2) * size_max / 2)

    def search_rect():
        best_score = -1
        best_rect = None
        for i in range(1000):
            r = random_rect()
            s = eval_rect(*r)
            if s > best_score:
                logging.debug('score update: to %d' % s)
                best_score = s
                best_rect = r
        return best_rect

    angles = np.arctan2(pts[:, 7], pts[:, 6])
    # plt.hist(angles, bins=50)
    # plt.show()
    h, bins = np.histogram(angles, bins=100)
    rot_max = bins[np.argmax(h)]

    ex = np.dot(rotation2d(rot_max), np.array([1, 0]))
    ey = np.dot(rotation2d(rot_max), np.array([0, 1]))
    xs = np.einsum('i, si -> s', ex, pts[:, :2])
    ys = np.einsum('i, si -> s', ey, pts[:, :2])

    xsh = np.histogram(xs, bins=50)
    ysh = np.histogram(ys, bins=50)
    xpeaks = []
    for pix in scipy.signal.argrelmax(xsh[0])[0]:
        xpeaks.append(xsh[1][pix])
    ypeaks = []
    for pix in scipy.signal.argrelmax(ysh[0])[0]:
        ypeaks.append(ysh[1][pix])

    logging.debug('Peaks: %dx%d' % (len(xpeaks), len(ypeaks)))

    rs = []
    for xp in itertools.combinations(xpeaks, 2):
        for yp in itertools.combinations(ypeaks, 2):
            center = np.dot(rotation2d(rot_max), [np.mean(xp), np.mean(yp)])
            halfsize = np.array([xp[1] - xp[0], yp[1] - yp[0]]) / 2
            if halfsize[0] < thresh_d or halfsize[1] < thresh_d:
                continue
            rs.append((rot_max, center, halfsize))
    #return random.sample(rs, 100)
    best_r = max(rs, key=lambda r:eval_rect(*r))
    logging.info("Best Score:%d" % eval_rect(*best_r))
    return [best_r]

    #r_org = random_rect()
    return [search_rect()]  # eval_rect(*r_org) + [r_org]


def squash_reg_error(pts, error_radius=0.2):
    """
    Smooth registration error using method in
    http://www-video.eecs.berkeley.edu/papers/elturner/thesis_paper.pdf
    Sec. 3.2

    pts: x, y, z, r, g, b, nx, ny, nz
    error_radius: max expected registration error
    """
    ortho_error_radius = error_radius * 0.3
    kdt = scipy.spatial.cKDTree(pts[:, :2])

    cos_thresh = math.cos(math.pi / 8)
    logging.debug("Removing noise")
    new_pts = []
    for p in pts:
        perp_vect = np.array([p[7], -p[6]])

        ixs = kdt.query_ball_point(p[:2], error_radius)
        if len(ixs) <= 1:
            new_pts.append(p)
        else:
            pts_subset = pts[ixs]
            cos = np.dot(pts_subset[:, 6:8], p[6:8])
            perp = np.abs(np.dot(pts_subset[:, :2] - p[:2], perp_vect))

            pts_subset = pts_subset[(cos > cos_thresh) & (perp < ortho_error_radius)]
            if len(pts_subset) <= 1:
                new_pts.append(p)
            else:
                new_pts.append(pts_subset.mean(axis=0))
    return np.array(new_pts)


def checkpoint(filename, process):
    """
    Return cached data if present.
    Otherwise, execute process, save to filename, and return result.
    """
    try:
        result = cPickle.load(open(filename, 'rb'))
        logging.info("Loaded from checkpoint %s" % filename)
        return result
    except (IOError, EOFError):
        logging.info("Checkpoint not found. Re-calculating")
        result = process()
        cPickle.dump(result, open(filename, 'wb'))
        return result


def do_everything(dir_path_in):
    cloud_big = load_cloud(os.path.join(dir_path_in, 'debug_points_interior.ply'))
    logging.info("Big cloud: %s" % str(cloud_big.shape))

    cloud = load_cloud(os.path.join(dir_path_in, 'debug_points_interior_slice.ply'))
    logging.info("Cloud shape: %s" % str(cloud.shape))

    xyzs = cloud[:, :3]

    # extract AABB and setup drawing region
    px_per_meter = 200
    x0 = xyzs[:, 0].min() - 1
    x1 = xyzs[:, 0].max() + 1
    y0 = xyzs[:, 1].min() - 1
    y1 = xyzs[:, 1].max() + 1

    width = int((x1 - x0) * px_per_meter)
    height = int((y1 - y0) * px_per_meter)

    surf = cairo.ImageSurface(cairo.FORMAT_RGB24, width, height)
    ctx = cairo.Context(surf)
    ctx.scale(px_per_meter, -px_per_meter)
    ctx.translate(-x0, -y1)
    # set bg to white
    ctx.set_source_rgb(1, 1, 1)
    ctx.paint()

    # global params
    step = 0.1

    # draw background grid
    # ctx.set_line_width(0.01)
    # for i in range(-100, 100):
    #     ctx.move_to(i * step, -10)
    #     ctx.line_to(i * step, 10)
    #     ctx.move_to(-10, i * step)
    #     ctx.line_to(10, i * step)
    # ctx.set_source_rgba(0, 0, 0, 0.3)
    # ctx.stroke()

    # Squash reg error
    cloud = checkpoint(
        os.path.join(dir_path_in, 'squash_cache.cache.pickle'),
        lambda: squash_reg_error(cloud))

    # bin to cells
    cells = {}
    for pt in cloud:
        ip = np.floor(pt[:2] / step)
        key = (int(ip[0]), int(ip[1]))
        cells.setdefault(key, []).append(pt)
    logging.info("#cells: %d" % len(cells))

    # throw away cells with too few points
    cell_ns = sorted(map(len, cells.values()))
    thresh_no = cell_ns[3 * len(cell_ns) // 4]
    logging.info('noise removal threshold: %d' % thresh_no)
    cells = {k: pts for (k, pts) in cells.items() if len(pts) >= thresh_no}
    logging.info("#cells (after noise removal): %d" % len(cells))

    # draw points in each cells
    n_len = 0.1
    for pts in cells.values():
        for pt in pts:
            x, y = pt[:2]
            nx, ny = pt[6 : 8]
            ctx.set_source_rgba(1, 0, 0, 0.1)
            ctx.set_line_width(0.01)
            ctx.move_to(x, y)
            ctx.rel_line_to(nx * n_len, ny * n_len)
            ctx.stroke()

            ctx.set_source_rgba(0, 0, 1, 0.03)
            ctx.arc(x, y, 0.01, 0, 2 * math.pi)
            ctx.fill()

    vmin = xyzs.min(axis=0)[:2]
    vmax = xyzs.max(axis=0)[:2]
    vsize = vmax - vmin
    # for i in range(100):
    #     p0 = np.random.rand(2) * vsize + vmin
    #     sz = np.random.rand(2) * (vsize * 0.5)
    #     th = np.random.rand(1) * math.pi / 2
    #     ctx.save()
    #     ctx.translate(*p0)
    #     ctx.rotate(th)
    #     ctx.rectangle(0, 0, sz[0], sz[1])
    #     ctx.set_source_rgba(0, 1, 0, 0.5)
    #     ctx.stroke()
    #     ctx.restore()
        
        # ctx.set_source_rgba(0, 1, 0, 0.1)
        # ctx.move_to(*p0[:2])
        # ctx.line_to(*p1[:2])
        # ctx.stroke()

    ccs = [cc for cc in to_4cc(cells.keys()) if len(cc) > 1]
    logging.info('#CCs: %d' % len(ccs))

    rev_cluster = {}
    for (ix, cc) in enumerate(ccs):
        logging.debug("%d: %d" % (ix, len(cc)))
        # Visualize region
        region_color = (0, np.random.rand(1), np.random.rand(1), 0.3)
        for k in cc:
            rev_cluster[k] = region_color
            # area
            ctx.rectangle(step * k[0], step * k[1], step, step)
            ctx.set_source_rgba(*region_color)
            ctx.fill()

        # Add label
        ctx.save()
        ctx.translate(step * k[0], step * k[1])
        ctx.scale(0.01, -0.01)
        ctx.set_source_rgb(0, 0, 0)
        ctx.show_text("cluster:%d" % ix)
        ctx.restore()

    #     # Fit rect
    #     pts = np.vstack([cells[k] for k in cc])
    #     print(pts.shape)
    #     for (rot, center, hsize) in fit_rect(pts):
    #         ctx.save()
    #         ctx.translate(*center)
    #         ctx.rotate(rot)
    #         ctx.rectangle(-hsize[0], -hsize[1], 2 * hsize[0], 2 * hsize[1])
    #         ctx.set_source_rgba(0, 0, 0, 0.8)
    #         ctx.stroke()
    #         ctx.restore()

    surf.write_to_png(os.path.join(dir_path_in, 'shapes.png'))

    logging.debug('VMin: %s' % xyzs.min(axis=0))
    z0 = xyzs[:, 2].min()
    for i in range(10):
        h0 = i * 0.25 + z0
        h1 = h0 + 0.25

        sl = cloud_big[(h0 < cloud_big[:, 2]) & (cloud_big[:, 2] < h1)]
        logging.debug(sl.shape)
        surf = cairo.ImageSurface(cairo.FORMAT_RGB24, width, height)
        ctx = cairo.Context(surf)
        ctx.scale(px_per_meter, -px_per_meter)
        ctx.translate(-x0, -y1)
        # set bg to white
        ctx.set_source_rgb(1, 1, 1)
        ctx.paint()

        for pt in sl:
            x, y = pt[:2]
            nx, ny = pt[6 : 8]
            ctx.set_source_rgba(1, 0, 0, 0.1)
            ctx.set_line_width(0.01)
            ctx.move_to(x, y)
            ctx.rel_line_to(nx * n_len, ny * n_len)
            ctx.stroke()

            ctx.set_source_rgba(0, 0, 1, 0.03)
            ctx.arc(x, y, 0.01, 0, 2 * math.pi)
            ctx.fill()

        surf.write_to_png(os.path.join(dir_path_in, 'shapes-%.1f.png' % h0))

    # do coloring
    for pt in cloud_big:
        ip = np.floor(pt[:2] / step)
        key = (int(ip[0]), int(ip[1]))

        if key in rev_cluster:
            pt[3] = rev_cluster[key][0] * 255
            pt[4] = rev_cluster[key][1] * 255
            pt[5] = rev_cluster[key][2] * 255
        else:
            # non-cluster
            pt[3] = 255
    serialize_points_as_ply(
        cloud_big,
        open(os.path.join(dir_path_in, "debug_shapes.ply"), 'w'))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="""
""",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '--input', required=True, type=str,
        help='Input scene asset bundle directory')

    args = parser.parse_args()
    logging.info("Processing %s" % args.input)

    chmod = 'sudo chmod a+rwx %s' % args.input
    logging.debug("Running %s" % chmod)
    os.system(chmod)
    do_everything(args.input)
