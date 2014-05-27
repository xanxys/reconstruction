from __future__ import print_function, division
import mist
import json

cloud = json.load(open('../ocha_points.json'))
print("Cloud size:%d head:%s" % (len(cloud), cloud[0]))

mesh = mist.construct_mesh(cloud)
print("Mesh #vert=%d #face=%d" % (len(mesh["vertices"]), len(mesh["faces"])))

json.dump(mesh, open('test_output_mesh.json', 'w'))
