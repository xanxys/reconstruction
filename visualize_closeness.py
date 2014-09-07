#!/bin/python2
# coding: utf-8
from __future__ import print_function, division
import matplotlib.pyplot as plt
import numpy as np
import json
import networkx as nx
import dateutil.parser


def get_sessions(fobj):
    """
    return {(session_id, session logs)}
    Entries with no session id will have "" as id.
    """
    log = [json.loads(line) for line in fobj]
    sessions = {}
    for entry in log:
        sid = entry.get("session_id", "")
        sessions.setdefault(sid, []).append(entry)
    return sessions

if __name__ == '__main__':
    for (sid, session) in get_sessions(open("log")).items():
        print('Processing sid=%s at %s' % (sid, session[0]["date"]))
        if sid == "":
            print('WARNING: old logs with empty sid found. Some results might be messed up.')

        # stats
        dur = dateutil.parser.parse(session[-1]["date"]) - dateutil.parser.parse(session[0]["date"])
        print('* Run duration: %s' % dur)

        plt.figure(1)

        # closeness matrix
        plt.subplot(121)
        target = filter(lambda e: e["msg"][0] == "closeness matrix", session)[0]
        closeness = target["msg"][1]
        closeness = np.array(closeness)

        x = plt.imshow(closeness, interpolation='none')
        plt.title('Pair distance (%s)' % target["date"])
        plt.xlabel('scan index')
        plt.ylabel('scan index')
        plt.colorbar(x)

        # Pairings
        plt.subplot(122)
        pairs = filter(lambda e: e["msg"][0] == "Pair:", session)
        pairs = [(pair["msg"][1], pair["msg"][2], pair["msg"][4]) for pair in pairs]
        g = nx.DiGraph()
        for (source0, source1, target) in pairs:
            g.add_edge(source0, target)
            g.add_edge(source1, target)
        p = nx.graphviz_layout(g, prog='twopi', args='')
        nx.draw(g, p, node_color='skyblue', alpha=0.8)

        # show all
        plt.show()
