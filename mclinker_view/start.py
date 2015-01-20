#!/bin/python2
from __future__ import print_function, division
import flask
import ujson as json
app = flask.Flask("visualizer")


@app.route('/')
def index():
    return flask.send_from_directory('./static', 'index.html')


@app.route('/static/<path>')
def serve_static(path):
    return flask.send_from_directory('./static', path)


@app.route('/data')
def api_data():
    result = {
        "status": "success",
        "data": json.load(open("../scene-home-tatami/link_mc.json"))
    }
    return flask.jsonify(**result)

app.run(debug=True)
