#!/bin/python3
from __future__ import print_function, division
import matplotlib.pyplot as plt
import numpy as np
import json
import networkx as nx

if __name__ == '__main__':
    dist = np.array(json.load(open("20141217-09:56-mc_dist.json")))
    conn_prob = np.exp(-dist / 0.01)

    # plt.imshow(conn_prob, interpolation='nearest')
    # plt.colorbar()
    # plt.show()

    n = len(dist)
    thresh = 0.05
    G = nx.Graph()
    G.add_nodes_from(range(n))
    for i in range(n):
        for j in np.arange(n)[dist[i] < thresh]:
            G.add_edge(i, j)
    
    ccs = nx.connected_components(G)

    # Modify MC in pklace
    link_mc = json.load(open("scene-small-gakusei/link_mc.json"))
    link_mc["groups"] = [[int(n) for n in cc] for cc in ccs]
    json.dump(link_mc, open("scene-small-gakusei/link_mc.json", "w"))
