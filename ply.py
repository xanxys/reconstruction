import numpy as np


def load_cloud_from_ply(fobj):
    """
    Return XYZRGBNormal (9, N) cloud
    Only works for certain data!

    Don't use after this experiment.
    """
    lines = list(fobj)
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
