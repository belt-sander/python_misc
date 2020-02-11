#!/usr/bin/python

from __future__ import print_function
import argparse
import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import matplotlib.pyplot as plt

def parse_args():
	arg_parser = argparse.ArgumentParser(description='simple visualization tool for xyzr data')
	arg_parser.add_argument('-r', '--rows', required=False, default=1000000, type=int, help='number of rows to process, default is 1000000')
	arg_parser.add_argument('-s1', '--scan_1', required=True, help='xyzr scan 1')
	arg_parser.add_argument('-s2', '--scan_2', required=True, help='xyzr scan 2')
	arg_parser.add_argument('-s3', '--scan_3', required=True, help='xyzr scan 3')
	arg_parser.add_argument('-s4', '--scan_4', required=True, help='xyzr scan 4')
	return arg_parser.parse_args()

def main():
	args = parse_args()
	scan1 = np.genfromtxt(args.scan_1)
	scan2 = np.genfromtxt(args.scan_2)
	scan3 = np.genfromtxt(args.scan_3)
	scan4 = np.genfromtxt(args.scan_4)
	one = np.zeros((len(scan1),1))
	zero = np.zeros((len(scan1),1))

	### scan 1 ###
	x1 = np.zeros((len(scan1),1))
	y1 = np.zeros((len(scan1),1))
	z1 = np.zeros((len(scan1),1))
	r1 = np.zeros((len(scan1),1))

	for i, row in enumerate(scan1):
		_x1 = row[0]
		_y1 = row[1]
		_z1 = row[2]
		_r1 = row[3]

		x1[i,:] = _x1
		y1[i,:] = _y1
		z1[i,:] = _z1 + 2 ### z height offset
		r1[i,:] = _r1 / 255.0
		one[i,:] = 1.0

		if i > args.rows:
			break

	### scan 2 ###
	x2 = np.zeros((len(scan1),1))
	y2 = np.zeros((len(scan1),1))
	z2 = np.zeros((len(scan1),1))
	r2 = np.zeros((len(scan1),1))

	for i, row in enumerate(scan2):
		_x = row[0]
		_y = row[1]
		_z = row[2]
		_r = row[3]

		x2[i,:] = _x
		y2[i,:] = _y
		z2[i,:] = _z + 2 ### z height offset
		r2[i,:] = _r / 255.0
		one[i,:] = 1.0

		if i > args.rows:
			break

	### scan 3 ###
	x3 = np.zeros((len(scan1),1))
	y3 = np.zeros((len(scan1),1))
	z3 = np.zeros((len(scan1),1))
	r3 = np.zeros((len(scan1),1))

	for i, row in enumerate(scan2):
		_x = row[0]
		_y = row[1]
		_z = row[2]
		_r = row[3] / 255.0

		x3[i,:] = _x
		y3[i,:] = _y
		z3[i,:] = _z + 2 ### z height offset
		r3[i,:] = _r 
		one[i,:] = 1.0

		if i > args.rows:
			break

	### scan 4 ###
	x4 = np.zeros((len(scan1),1))
	y4 = np.zeros((len(scan1),1))
	z4 = np.zeros((len(scan1),1))
	r4 = np.zeros((len(scan1),1))

	for i, row in enumerate(scan2):
		_x = row[0]
		_y = row[1]
		_z = row[2]
		_r = row[3] / 255.0

		x4[i,:] = _x
		y4[i,:] = _y
		z4[i,:] = _z + 2 ### z height offset
		r4[i,:] = _r 
		one[i,:] = 1.0

		if i > args.rows:
			break

	### rotation tests ###
	'''
	reference frame of lidar (velarray) is:
	x == right
	y == forward
	z == up
	'''
	rows = np.zeros((len(scan1),1))
	x1_rot = np.zeros((len(scan1),1))
	y1_rot = np.zeros((len(scan1),1))
	z1_rot = np.zeros((len(scan1),1))
	pre_rotation_data = np.column_stack((x1, y1, z1))
	for i, row in enumerate(pre_rotation_data):
		rows[i,:] = i
		_roll = 10 * np.pi/180 # degrees to radians
		_pitch = 10 * np.pi/180 # degrees to radians
		_yaw = 25 * np.pi/180 # degrees to radians
		_x1 = row[0]
		_y1 = row[1]
		_z1 = row[2]

		_x1_roll = (np.sin(_roll) * _z1) + (np.cos(_roll) * _x1)
		_z1_roll = (np.cos(_roll) * _z1) + (-np.sin(_roll) * _x1)
		_y1_roll = _y1 
		_x1_roll_delta = _x1 - _x1_roll
		_y1_roll_delta = _y1 - _y1_roll
		_z1_roll_delta = _z1 - _z1_roll

		_x1_pitch = _x1
		_y1_pitch = (np.sin(_pitch) * _z1) + (np.cos(_pitch) * _y1)
		_z1_pitch = (np.cos(_pitch) * _z1) + (-np.sin(_pitch) * _y1) 
		_x1_pitch_delta = _x1 - _x1_pitch
		_y1_pitch_delta = _y1 - _y1_pitch
		_z1_pitch_delta = _z1 - _z1_pitch

		_x1_yaw = (np.cos(_yaw) * _x1) + (-np.sin(_yaw) * _y1) 
		_y1_yaw = (np.sin(_yaw) * _x1) + (np.cos(_yaw) * _y1)
		_z1_yaw = _z1		
		_x1_yaw_delta = _x1 - _x1_yaw
		_y1_yaw_delta = _y1 - _y1_yaw
		_z1_yaw_delta = _z1 - _z1_yaw

		x1_rot[i,:] = _x1_roll_delta + _x1_pitch_delta + _x1_yaw_delta + _x1
		y1_rot[i,:] = _y1_roll_delta + _y1_pitch_delta + _y1_yaw_delta + _y1
		z1_rot[i,:] = _z1_roll_delta + _z1_pitch_delta + _z1_yaw_delta + _z1

		### individual rotations ###
		# roll rotation (correct)
		# x1_rot[i,:] = (np.sin(_roll) * _z1) + (np.cos(_roll) * _x1)
		# y1_rot[i,:] = _y1
		# z1_rot[i,:] = (np.cos(_roll) * _z1) + (-np.sin(_roll) * _x1) 

		# pitch rotation (correct)
		# x1_rot[i,:] = _x1
		# y1_rot[i,:] = (np.sin(_pitch) * _z1) + (np.cos(_pitch) * _y1)
		# z1_rot[i,:] = (np.cos(_pitch) * _z1) + (-np.sin(_pitch) * _y1) 

		# yaw rotation (correct)
		# y1_rot[i,:] = (np.sin(_yaw) * _x1) + (np.cos(_yaw) * _y1)
		# x1_rot[i,:] = (np.cos(_yaw) * _x1) + (-np.sin(_yaw) * _y1) 
		# z1_rot[i,:] = _z1

	### concatanate scan daterz ###
	x_y_z_1 = np.concatenate((x1, y1, z1), axis=1)
	reflectivity_1 = np.concatenate((one, zero, zero, r1), axis=1)

	x_y_z_1_rot = np.concatenate((x1_rot, y1_rot, z1_rot), axis=1) # rotation test
	reflectivity_1_rot = np.concatenate((zero, one, zero, r1), axis=1) # rotation test

	x_y_z_2 = np.concatenate((x2, y2, z2), axis=1)
	reflectivity_2 = np.concatenate((zero, one, zero, r2), axis=1)

	x_y_z_3 = np.concatenate((x3, y3, z3), axis=1)
	reflectivity_3 = np.concatenate((zero, zero, one, r3), axis=1)

	x_y_z_4 = np.concatenate((x4, y4, z4), axis=1)
	reflectivity_4 = np.concatenate((one, one, zero, r4), axis=1)

	# open app
	app=pg.QtGui.QApplication([])

	# array of data
	plt_1 = gl.GLScatterPlotItem(pos=x_y_z_1, color=reflectivity_1, size=2)
	plt_1_rot = gl.GLScatterPlotItem(pos=x_y_z_1_rot, color=reflectivity_1_rot, size=2)

	plt_2 = gl.GLScatterPlotItem(pos=x_y_z_2, color=reflectivity_2, size=2)
	plt_3 = gl.GLScatterPlotItem(pos=x_y_z_3, color=reflectivity_3, size=2)
	plt_4 = gl.GLScatterPlotItem(pos=x_y_z_4, color=reflectivity_4, size=2)

	# add widget / plot data	
	w = gl.GLViewWidget()
	w.addItem(plt_1)
	w.addItem(plt_1_rot)

	# w.addItem(plt_2)
	# w.addItem(plt_3)
	# w.addItem(plt_4)

	# add grid
	g = gl.GLGridItem()
	w.addItem(g)
	
	# show
	w.show()
	pg.QtGui.QApplication.exec_()

if __name__=='__main__':
	main()
