#!/bin/python2
from __future__ import print_function, division
import json
import sys

if __name__ == '__main__':
    in_obj = json.load(sys.stdin)
    json.dump(in_obj, sys.stdout)
