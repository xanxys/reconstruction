#!/bin/env python
from __future__ import print_function, division
import flask

app = flask.Flask(__name__)

@app.route('/')
def hello():
	return 'Hello World!'

if __name__ == '__main__':
	app.run()
