import argparse
import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R


def parse_args():
	arg_parser = argparse.ArgumentParser(description='simple visualization tool for xyzr data')
	arg_parser.add_argument('-row', '--rows', required=False, default=1000000, type=int, help='number of rows to process, default is 1000000')
	arg_parser.add_argument('-s1', '--scan_1', required=True, help='xyzr scan 1')
	arg_parser.add_argument('-r', '--roll', type=int, required=True, help='roll in degrees')
	arg_parser.add_argument('-p', '--pitch', type=int, required=True, help='pitch in degrees')
	arg_parser.add_argument('-y', '--yaw', type=int,  required=True, help='yaw in degrees')
	return arg_parser.parse_args()

def main():
	args = parse_args()
	scan1 = np.genfromtxt(args.scan_1)

	### scan 1 ###
	x1 = np.zeros((len(scan1),1))
	y1 = np.zeros((len(scan1),1))
	z1 = np.zeros((len(scan1),1))
	r1 = np.zeros((len(scan1),1))

	x1_rot = np.zeros((len(scan1),1))
	y1_rot = np.zeros((len(scan1),1))
	z1_rot = np.zeros((len(scan1),1))	
	
	one1 = np.zeros((len(scan1),1))
	zero1 = np.zeros((len(scan1),1))

	for i, row in enumerate(scan1):
		_x1 = row[0]
		_y1 = row[1]
		_z1 = row[2]
		_r1 = row[3]

		### rotation tests ###
		'''
		reference frame of lidar (velarray) is:
		x == right
		y == forward
		z == up
		'''

		_point_array = np.array([[_x1],[_y1],[_z1]]) # yaw, pitch, roll		
		_rotation = R.from_euler('zxy', [args.yaw, args.pitch, args.roll], degrees=True)		
		_rotation_matrix = _rotation.as_matrix()	
		_rotated_output = np.matmul(_rotation_matrix, _point_array)		
		_rotated_point_array = np.reshape(_rotated_output, len(_rotated_output))

		x1[i,:] = _x1
		y1[i,:] = _y1
		z1[i,:] = _z1 + 2 ### z height offset
		r1[i,:] = _r1 / 255.0
		one1[i,:] = 1.0

		z1_rot[i,:] = _rotated_point_array[2] + 2 ### z height offset
		x1_rot[i,:] = _rotated_point_array[0]
		y1_rot[i,:] = _rotated_point_array[1]

		if i > args.rows:
			break

	pre_rotation_data = np.column_stack((x1, y1, z1))

	### concatanate scan daterz ###
	x_y_z_1 = np.concatenate((x1, y1, z1), axis=1)
	reflectivity_1 = np.concatenate((one1, zero1, zero1, r1), axis=1)

	x_y_z_1_rot = np.concatenate((x1_rot, y1_rot, z1_rot), axis=1) # rotation test
	reflectivity_1_rot = np.concatenate((zero1, one1, zero1, r1), axis=1) # rotation test

	# open app
	app = pg.QtGui.QApplication([])

	# array of data
	plt_1 = gl.GLScatterPlotItem(pos=x_y_z_1, color=reflectivity_1, size=4)
	plt_1_rot = gl.GLScatterPlotItem(pos=x_y_z_1_rot, color=reflectivity_1_rot, size=4)

	# add widget / plot data	
	w = gl.GLViewWidget()
	w.addItem(plt_1)
	w.addItem(plt_1_rot)

	# add grid
	g = gl.GLGridItem()
	w.addItem(g)
	
	# show
	w.show()
	pg.QtGui.QApplication.exec_()

if __name__=='__main__':
	main()
