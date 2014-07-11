#!/bin/python3
import cairo
import itertools
import math
import numpy as np
import numpy.linalg as la
import random

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
		((calc_distance(path), path) for path in itertools.permutations(points)),
		key=lambda pair: pair[0])

def random_swap(ls):
	ls = list(ls)  # copy
	n = len(ls)
	# Sample two different indices
	i = random.randint(0, n - 2)
	j = random.randint(i + 1, n - 1)
	ls[i], ls[j] = ls[j], ls[i]
	return ls

def approx_tsp(points, num_iteration = 100):
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
		#print("%dth iteration: temp=%.1f delta=%.1f" % (i, temperature, delta))
		#print("current cost = %.1f" % calc_distance(path))
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
		r = (np.random.rand(2) * 2 - 1, np.random.rand(2) + 0.1)
		if len(rects) == 0 or any(overlap2d(r_some, r) for r_some in rects):
			rects.append(r)
	return rects


def test_concave_wrapping():
	rects = generate_overlapping_rects(3)

	# Setup a context that maps [-2,2]^2 region to an image
	size = 500
	surf = cairo.ImageSurface(cairo.FORMAT_RGB24, size, size)
	ctx = cairo.Context(surf)
	ctx.scale(size / 4, size / 4)
	ctx.translate(2, 2)
	ctx.set_source_rgb(1, 1, 1)
	ctx.paint()
	ctx.set_line_width(0.01)

	# visualize rectangles
	ctx.set_source_rgb(0, 0, 0)
	for r in rects:
		ctx.rectangle(r[0][0], r[0][1], r[1][0], r[1][1])
		ctx.stroke()

	# put points in rectangles
	points = np.concatenate(
		[r[0] + np.random.rand(100, 2) * r[1] for r in rects],
		axis = 0)

	# visualize points
	ctx.set_source_rgba(0, 0, 1, 0.5)
	for pt in points:
		ctx.arc(pt[0], pt[1], 0.01, 0, 2 * math.pi)
		ctx.fill()

	surf.write_to_png("sa_wrap.png")


if __name__ == '__main__':
	# compare_tsp_approx_vs_exact()
	test_concave_wrapping()
