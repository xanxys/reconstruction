#!/bin/env python
from __future__ import print_function, division
import flask

import os
import cv2

app = flask.Flask(__name__)

dataset_root = '/data-new/research/2014/reconstruction/NYU2-slice'

@app.route('/')
def hello():
	ls = ''
	for d in os.listdir(dataset_root):
		ls += '<img src="/%s">' % d
	return 'List of Images' + ls

@app.route('/<scene_id>')
def image(scene_id):
	scene_root = os.path.join(dataset_root, scene_id)
	[rgb] = [f for f in os.listdir(scene_root) if f.endswith('.ppm')]
	rgb_image = cv2.imread(os.path.join(scene_root, rgb))

	binary = cv2.imencode('.jpg', rgb_image)[1].tostring()
	resp = flask.make_response(binary)
	resp.headers['Content-Type'] = 'image/jpeg'
	return resp

if __name__ == '__main__':
	app.run(debug=True)
