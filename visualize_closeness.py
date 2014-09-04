# coding: utf-8
import matplotlib.pyplot as plt
import numpy as np
import json

log = [json.loads(line) for line in open('log')]

closeness = filter(lambda e: e["msg"][0]=="closeness matrix", log)[0]["msg"][1]
closeness = np.array(closeness)

x = plt.imshow(closeness, interpolation='none')
plt.colorbar(x)
plt.show()
