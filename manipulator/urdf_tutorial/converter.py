#!/usr/bin/env python

import json
import math
import mathutils
import numpy

theta4 = 0
len = 0.2
angle = 1.56
axis_x = (1, 0, 0)
axis_z = (0, 0, 1)
with open('DH.json','r') as file:
	params = json.loads(file.read())

with open('urdf/urdf.yaml','w') as file:
	for key in params.keys():

		a, d, alpha, theta = params[key]
		a = float(a)
		d = float(d)
		alpha = float(alpha)
		theta = float(theta)

		axis_x = (1, 0, 0)
		axis_z = (0, 0, 1)
		rad = 0.1

		rot_x = mathutils.Matrix.Rotation(alpha,4, 'X')
		trans_x = mathutils.Matrix.Translation((a,0,0))
		rot_z = mathutils.Matrix.Rotation(theta,4, 'Z')
		trans_z = mathutils.Matrix.Translation((0,0,d))

		dh = rot_x @ trans_x @ rot_z @ trans_z
		rpy = dh.to_euler()
		xyz = dh.to_translation()
		
		if key=="i1":
			rad=rad/2
			

		file.write(key + ":\n")
		if key=="i1" or key=="i2":
			xyz[2]=xyz[2]+len

		file.write(" joint_xyz: {} {} {}\n".format(*xyz))
		file.write(" joint_rpy: {} {} {}\n".format(*rpy))
		file.write(" link_xyz: 0 0 0 \n")
		file.write(" link_rpy: 0 0 0 \n")
		file.write(" link_length: {} \n".format(len))
		file.write(" radius: {} \n".format(rad))


