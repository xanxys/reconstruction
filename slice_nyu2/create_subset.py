#!/bin/env python3
import json
import os
import shutil

source = '/data-new/research/2014/reconstruction/NYU2'
dest = '/data-new/research/2014/reconstruction/NYU2-slice'

def get_minimum_subset(scene_dir):
	"""
	scene_dir: absolute path of a scene directory.
	return: list of relative paths suitable for single-frame reconstruction.

	Algorithm used is similar to
	https://bitbucket.org/xanxys/reconstruction/src/244f82f6b51b2feb4ea1bd72549fb198e7b48e2e/data_source.cpp?at=master#cl-183
	"""
	time_tolerance = 0.1

	def time_of(path_rel):
		return float(path_rel.split('-')[1])

	# Partition files
	rgbs = []
	depths = []
	for path in os.listdir(scene_dir):
		ext = os.path.splitext(path)[1]
		if ext == '.ppm':
			rgbs.append(path)
		elif ext == '.pgm':
			depths.append(path)
	# Find corresponding frames by
	# 1. arbitrarily choosing RGB frame
	# 2. find closest match
	the_rgb = rgbs[0]
	t_rgb = time_of(the_rgb)

	the_depth = min(depths, key = lambda depth: abs(time_of(depth) - t_rgb))
	delta = abs(time_of(the_depth) - t_rgb)

	if delta < time_tolerance:
		return [the_rgb, the_depth]
	else:
		raise RuntimeError("Corresponding frame not found")

# Recreate directory
if os.path.exists(dest):
	raise RuntimeError(
		"Destination %s not empty; make sure it's garbage and remove manually" %
		dest)
os.mkdir(dest)

count_dir = 0
count_file = 0
count_dir_total = 0
for scene_dir in os.listdir(source):
	count_dir_total += 1
	try:
		scene_dir_src = os.path.join(source, scene_dir)
		paths_rel = get_minimum_subset(scene_dir_src)
		print('%s -> %s' % (scene_dir, paths_rel))

		scene_dir_dst = os.path.join(dest, scene_dir)
		os.mkdir(scene_dir_dst)
		for path_rel in paths_rel:
			try:
				shutil.copy(
					os.path.join(scene_dir_src, path_rel),
					os.path.join(scene_dir_dst, path_rel))
			except PermissionError:
				pass  # HACK: samba don't support permissions
			count_file += 1
		count_dir += 1
	except:
		pass

print('count_dir_total: ', count_dir_total)
print('count_dir: ', count_dir)
print('count_file: ', count_file)

