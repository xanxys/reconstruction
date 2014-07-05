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

for n in range(2, 10):
	print("## Planar TSP with N = %d" % n)
	pts = np.random.rand(n, 2)
	print('Approx:', approx_tsp(pts))
	print('Exact:', solve_tsp(pts))
