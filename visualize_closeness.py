#!/bin/python2
# coding: utf-8
import matplotlib.pyplot as plt
import numpy as np
import json

if __name__ == '__main__':
    log = [json.loads(line) for line in open('log')]

    targets = filter(lambda e: e["msg"][0] == "closeness matrix", log)

    for target in targets:
        print('Processing %s' % target["date"])
        closeness = target["msg"][1]
        closeness = np.array(closeness)

        x = plt.imshow(closeness, interpolation='none')
        plt.title('Pair distance (%s)' % target["date"])
        plt.xlabel('scan index')
        plt.ylabel('scan index')
        plt.colorbar(x)
        plt.show()
