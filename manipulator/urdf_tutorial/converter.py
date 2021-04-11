#!/usr/bin/env python

import json
import math
import mathutils
import numpy

theta4 = 0
d = 0.2
rad_s = 0.05
rad_m = 0.1
axis_x = (1, 0, 0)
axis_z = (0, 0, 1)
with open('DH.json','r') as file:
	params = json.loads(file.read())

with open('urdf2.yaml','w') as file:
	for key in params.keys():

		a, d, alpha, theta = params[key]
		a = float(a)
		d = float(d)
		alpha = float(alpha)
		theta = float(theta)

		axis_x = (1, 0, 0)
		axis_z = (0, 0, 1)
		
		if key==i3:
			theta4 = theta
			theta = 0

		rot_x = mathutils.Matrix.Rotation(alpha,4,'X')
		trans_x = mathutils.Matrix.Translation((a,0,0))
		rot_z = mathutils.Matrix.Rotation(theta,4,'Z')
		trans_z = mathutils.Matrix.Translation((0,0,d))

		dh = trans_x @ rot_x @ rot_z @ trans_z
		rpy = dh.to_euler()
		xyz = dh.to_translation()
		
		if theta ==0:
			rad = rad_s
		else:
			rad = rad_m

		file.write(key + ":\n")
		file.write(" joint_xyz=\"{} {} {}\"\n".format(*xyz))
		file.write(" joint_rpy=\"{} {} {}\"\n".format(*rpy))
		file.write(" link_xyz: 0 0 0 \n")
		file.write(" link_rpy: 0 0 0 \n")
		file.write(" link_length: {} \n".format(d))
		file.write(" radius: {} \n".format(rad))

	a, d, alpha = 0.0
	theta = theta4
	rot_x = mathutils.Matrix.Rotation(alpha,4,'X')
	trans_x = mathutils.Matrix.Translation((a,0,0))
	rot_z = mathutils.Matrix.Rotation(theta,4,'Z')
	trans_z = mathutils.Matrix.Translation((0,0,d))

	dh = trans_x @ rot_x @ rot_z @ trans_z
	rpy = dh.to_euler()
	xyz = dh.to_translation()
	if theta ==0:
		rad = rad_s
	else:
		rad = rad_m
	file.write("i4" + ":\n")
	file.write("joint_xyz=\"{} {} {}\"\n".format(*xyz))
	file.write("joint_rpy=\"{} {} {}\"\n".format(*rpy))
	file.write(" link_xyz: 0 0 0 \n")
	file.write(" link_rpy: 0 0 0 \n")
	file.write(" link_length: {} \n".format(d))
	file.write(" radius: {} \n".format(rad))
