#!/bin/env python
from __future__ import print_function, division
import flask

import cv2
import imp
import os
import traceback

import json

slic = imp.load_dynamic('slic', 'SLIC-Superpixels/slic.so')

app = flask.Flask(__name__, static_url_path='/py_static')

dataset_root = '/data-new/research/2014/reconstruction/NYU2-slice'


class PointCloud(object):
	def __init__(self, path):
		self.points = json.load(open(path))

	def color(self, color):
		""" Assign single color to all points. """
		for v in self.points:
			v["r"] = color["r"]
			v["g"] = color["g"]
			v["b"] = color["b"]
		return self

	def serializeJson(self):
		return {
			"type": "pointcloud",
			"data": self.points
		}

class Mesh(object):
	def __init__(self, path):
		self.points = json.load(open(path))

	def serializeJson(self):
		return {
			"type": "mesh",
			"data": self.points
		}

# TODO: separate server & interactive interpreter namespace
def load_point_cloud(path):
	return PointCloud(path)

def load_mesh(path):
	return Mesh(path)



@app.route('/')
def index():
	return flask.send_from_directory('./py_static', 'index.html')

@app.route('/static/<path:path>')
def serve_static(path):
	return flask.send_from_directory('./py_static', path)

@app.route('/snippet', methods=['POST'])
def post_snippet():
	try:
		result = eval(flask.request.values['code'])
		json_result = None
		if hasattr(result, 'serializeJson'):
			try:
				json_result = result.serializeJson()
			except:
				pass

		ext_result = {
			'type': 'success',
			'str': str(result),
			'json': json_result
		}
	except:
		ext_result = {
			'type': 'exception',
			'str': traceback.format_exc()
		}

	return flask.jsonify(**ext_result)

@app.route('/<scene_id>')
def image(scene_id):
	scene_root = os.path.join(dataset_root, scene_id)
	[rgb] = [f for f in os.listdir(scene_root) if f.endswith('.ppm')]
	rgb_image = cv2.imread(os.path.join(scene_root, rgb))

	rgb_image = slic.oversegmentate(rgb_image)

	binary = cv2.imencode('.jpg', rgb_image)[1].tostring()
	resp = flask.make_response(binary)
	resp.headers['Content-Type'] = 'image/jpeg'
	return resp

if __name__ == '__main__':
	app.run(debug=True, port=5321)
