#!/usr/bin/env python

import json
import math
import mathutils
import numpy

with open('DH.json','r') as file:
	params = json.loads(file.read())

with open('urdf.yaml','w') as file:
	for key in params.keys():

		a, d, alpha, theta = params[key]
		a = float(a)
		d = float(d)
		alpha = float(alpha)
		theta = float(theta)

		axis_x = (1, 0, 0)
		axis_z = (0, 0, 1)

		rot_x = mathutils.Matrix.Rotation(alpha,4,'X')
		trans_x = mathutils.Matrix.Translation((a,0,0))
		rot_z = mathutils.Matrix.Rotation(theta,4,'Z')
		trans_z = mathutils.Matrix.Translation((0,0,d))

		dh = trans_x @ rot_x @ rot_z @ trans_z
		rpy = dh.to_euler()
		xyz = dh.to_translation()

		file.write(key + ":\n")
		file.write("xyz=\"{} {} {}\"\n".format(*xyz)) #komentarz na przyszlosc, ogarnac nazwy
		file.write("rpy=\"{} {} {}\"\n".format(*rpy))
		file.write(" link_xyz: {} 0 0 \n".format(xyz[0]/2))
		file.write(" link_rpy: 0 0 0 \n")
		file.write(" link_length: {} \n\n".format(a))
