#!/bin/python
from __future__ import print_function, division
import numpy as np
import scipy.linalg as la
import scipy.optimize
import random
import json
import sys


def derivative(f, epsilon=1e-3):
    """
    Return numerical gradient of multivariate function f.
    """
    def dfdx(x):
        n = len(x)
        return np.array(
            [f(x + dx) - f(x - dx) for dx in np.eye(n)]) / (2 * epsilon)
    return dfdx


def test_random():
    n_loc = 5
    locs = []
    for l in range(n_loc):
        # (value, #samples)
        n = 15
        data = []
        for i in range(n):
            if random.random() < 0.1:
                data.append((0, 0))
            else:
                data.append((random.gauss(100, 20), random.uniform(100, 1000)))
        data = np.array(data)
        locs.append(data)
    print(locs)

    def cost(ms_l):
        """
        ms_l: color multiplier + Lagrange multiplier
        """
        ms = ms_l[:-1]
        lam = ms_l[-1]
        j = 0
        for data in locs:
            vals_multiplied = data[:, 0] * ms
            pairs_weight = np.outer(data[:, 1], data[:, 1])
            pairs_diff = (vals_multiplied[np.newaxis, :] - vals_multiplied[:, np.newaxis]) ** 2
            pairs_cost = pairs_weight * pairs_diff
            j += pairs_cost.sum()
        return j + lam * (ms.sum() / len(ms) - 1)

    ms = np.ones([n])
    ms_l = np.hstack([ms, [1]])
    print("initial", ms_l, cost(ms_l))
    ms_l = scipy.optimize.fsolve(derivative(cost), ms_l)
    print(ms_l[:-1])


def calculate_color_multipliers(din):
    n_scan = len(din[0])
    multipliers = []
    for channel in range(3):
        locs = []
        for l_ls in din:
            data = []
            for (color, samples) in l_ls:
                data.append((color[channel], samples))
            locs.append(np.array(data))

        def cost_r(ms):
            j = 0
            for data in locs:
                vals_multiplied = data[:, 0] * ms
                pairs_weight = np.outer(data[:, 1], data[:, 1])
                pairs_diff = (
                    vals_multiplied[np.newaxis, :] -
                    vals_multiplied[:, np.newaxis]) ** 2
                pairs_cost = pairs_weight * pairs_diff
                j += pairs_cost.sum()
            return j

        ms = np.ones([n_scan])
        result = scipy.optimize.fmin_l_bfgs_b(
            cost_r, ms, approx_grad=True,
            bounds=[(0.1, 10) for x in range(n_scan)])
        ms = result[0]
        multipliers.append(ms)
    multipliers = np.array(multipliers).T
    assert(multipliers.shape == (n_scan, 3))
    # normalize
    multipliers /= (multipliers.sum(axis=0)[np.newaxis, :] / n_scan)

    return [[float(e) for e in row] for row in multipliers]


# input: [[((Float, Float, Float), Float)]]
#  per-location, per-scan, (value, #samples)
# output: [(Float, Float, Float)]
#  color multipliers
if __name__ == '__main__':
    in_obj = json.load(sys.stdin)
    out_obj = calculate_color_multipliers(in_obj)
    json.dump(out_obj, sys.stdout)
