#!/bin/python2
from __future__ import print_function, division
import flask
import ujson as json
import os
import os.path
import random
app = flask.Flask("visualizer")

capturer_data_path = "../../capturer/proc_data"

@app.route('/')
def index():
    return flask.send_from_directory('./static', 'index.html')


@app.route('/static/<path>')
def serve_static(path):
    return flask.send_from_directory('./static', path)


@app.route('/list')
def api_list():
    result = {
        "status": "success",
        "data": os.listdir(capturer_data_path)
    }
    return flask.jsonify(**result)

@app.route('/data/<name>')
def api_data(name):
    path = os.path.join(capturer_data_path, name)
    es = os.listdir(path)
    print('Data path %s / #scan=%d' % (path, len(es)))

    v_data = {
        "scans": []
    }
    for e in es:
        points = json.load(open(os.path.join(path, e, "points.json")))
        n_points = len(points)
        sub_points = random.sample(points, n_points // 5)
        v_data["scans"].append({
            "cloud": sub_points,
            "name": e
        })

    result = {
        "status": "success",
        "data": v_data
    }
    return flask.jsonify(**result)

app.run(debug=True)
