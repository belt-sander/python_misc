#!/usr/bin/env python

###
### Tool to plot, convert, and output data to be used by the kitti odometry_evalation.cpp tool
###

from __future__ import print_function
import argparse
import numpy as np
import matplotlib.pyplot as plt
import tf_conversions
from pyproj import Proj

def parse_args():
	arg_parser = argparse.ArgumentParser(description='basic AF plotting script')
	arg_parser.add_argument('-s', '--slamInput', required=True, help='input file from slamOdomToText.py')
	arg_parser.add_argument('-g', '--groundInput', required=True, help='input file from oxts_poses file')
	arg_parser.add_argument('-so', '--slamOutput', required=False, help='slam output in kitti 3x4 translation matrix')
	arg_parser.add_argument('-go', '--groundOutput', required=False, help='gt output in kitti 3x4 translation matrix')

	return arg_parser.parse_args()

def main():
	args = parse_args()
	slam_input = np.genfromtxt(args.slamInput, delimiter=',')
	ground_input = np.genfromtxt(args.groundInput, delimiter=',', skip_header=1)
	print('slam poses: ', len(slam_input), ' ground poses: ', len(ground_input))

	###
	### Convert "x y z" from oxts_poses from NWU (actual) to XYZ (actual)	
	###

	ground_time = ground_input[:,10]
	ground_n = ground_input[:,3]
	ground_w = ground_input[:,4]
	ground_u = ground_input[:,5]
	ground_x = np.zeros((len(ground_input),1))
	ground_y = np.zeros((len(ground_input),1))
	ground_z = np.zeros((len(ground_input),1))

	ground_r_pose_init = []
	ground_roll =  np.zeros((len(ground_input),1))
	ground_pitch =  np.zeros((len(ground_input),1))
	ground_yaw =  np.zeros((len(ground_input),1))

	for i, row in enumerate(ground_input):
		### output is NWU frame as output of pose-graph mapping pipeline ###
		_n = row[3]
		_w = row[4]
		_u = row[5]

		### Establish transformation matrix (rotation and translation) ###
		# Get rotation matrix from quaternions and add in translation to righthand side of matrix
		ground_r_matrix = tf_conversions.transformations.quaternion_matrix([row[6], row[7], row[8], row[9]])
		ground_r_matrix[0,3] = _n
		ground_r_matrix[1,3] = _w
		ground_r_matrix[2,3] = _u

		# Take inverse of transformation matrix
		ground_r_matrix_inv = np.linalg.inv(ground_r_matrix)
		ground_r_pose_init.append(ground_r_matrix_inv)

		# Multiply inv transformation matrix by position vector
		ground_actual_matrix = np.matmul(ground_r_pose_init[0], ground_r_matrix) # pose[i] = pose[0].inverse() * pose[i]
		ground_x[i,:] = ground_actual_matrix[0,3]
		ground_y[i,:] = ground_actual_matrix[1,3]
		ground_z[i,:] = ground_actual_matrix[2,3]

		_00 = ground_actual_matrix[0,0]
		_01 = ground_actual_matrix[0,1]
		_02 = ground_actual_matrix[0,2]
		_10 = ground_actual_matrix[1,0]
		_11 = ground_actual_matrix[1,1]
		_12 = ground_actual_matrix[1,2]
		_20 = ground_actual_matrix[2,0]
		_21 = ground_actual_matrix[2,1]
		_22 = ground_actual_matrix[2,2]

		ground_euler = tf_conversions.transformations.euler_from_matrix([[_00, _01, _02, 0], [_10, _11, _12, 0], [_20, _21, _22, 0], [0,0,0,1]])
		ground_roll[i,:] = ground_euler[0] 
		ground_pitch[i,:] = ground_euler[1]
		ground_yaw[i,:] = ground_euler[2] 

	###
	### Input raw output of slam/lidar_rectifier/odometry topic
	###

	slam_time = slam_input[:,10]
	slam_x = slam_input[:,3]
	slam_y = slam_input[:,4]
	slam_z = slam_input[:,5]
	slam_roll = np.zeros((len(slam_input),1))
	slam_pitch = np.zeros((len(slam_input),1))
	slam_yaw = np.zeros((len(slam_input),1))

	the_slam_file = open(args.slamOutput, 'a')

	for i, row in enumerate(slam_input):
		slam_euler = tf_conversions.transformations.euler_from_quaternion([row[6], row[7], row[8], row[9]])
		slam_roll[i,:] = slam_euler[0]
		slam_pitch[i,:] = slam_euler[1]
		slam_yaw[i,:] = slam_euler[2]

		slam_r_matrix = tf_conversions.transformations.quaternion_matrix([row[6], row[7], row[8], row[9]])
		_00 = slam_r_matrix[0,0]
		_01 = slam_r_matrix[0,1]
		_02 = slam_r_matrix[0,2]

		slam_r_matrix[0,3] = slam_x[i]
		_03 = slam_r_matrix[0,3] 

		_10 = slam_r_matrix[1,0]
		_11 = slam_r_matrix[1,1]
		_12 = slam_r_matrix[1,2]

		slam_r_matrix[1,3] = slam_y[i]
		_13 = slam_r_matrix[1,3] 

		_20 = slam_r_matrix[2,0]
		_21 = slam_r_matrix[2,1]
		_22 = slam_r_matrix[2,2]

		slam_r_matrix[2,3] = slam_z[i]
		_23 = slam_r_matrix[2,3]

		_r_kitti = np.array([[0,-1,0,0],[0,0,-1,0],[1,0,0,0],[0,0,0,1]])
		_kitti_frame = np.matmul(_r_kitti, slam_r_matrix)
		_r00 = _kitti_frame[0,0]
		_r01 = _kitti_frame[0,1]
		_r02 = _kitti_frame[0,2]
		_r03 = _kitti_frame[0,3]
		_r10 = _kitti_frame[1,0]
		_r11 = _kitti_frame[1,1]
		_r12 = _kitti_frame[1,2]
		_r13 = _kitti_frame[1,3]
		_r20 = _kitti_frame[2,0]
		_r21 = _kitti_frame[2,1]
		_r22 = _kitti_frame[2,2]
		_r23 = _kitti_frame[2,3]

		output_data = [str(_r00), str(_r01), str(_r02), str(_r03), str(_r10), str(_r11), str(_r12), str(_r13), str(_r20), str(_r21), str(_r22), str(_r23)]     
		the_slam_file.write(" ".join(output_data) + '\n')

	the_slam_file.close()

	###
	### Pose to pose evaluation
	###

	g_int_x = np.zeros((len(slam_input),1))
	g_int_y = np.zeros((len(slam_input),1))
	g_int_z = np.zeros((len(slam_input),1))
	g_int_roll = np.zeros((len(slam_input),1))
	g_int_pitch = np.zeros((len(slam_input),1))
	g_int_yaw = np.zeros((len(slam_input),1))

	the_file = open(args.groundOutput, 'a')

	for i in range(len(slam_input)):
		g_int_x[i,:] = (np.interp(slam_time[i], ground_time, ground_x.flatten()))
		g_int_y[i,:] = (np.interp(slam_time[i], ground_time, ground_y.flatten()))
		g_int_z[i,:] = (np.interp(slam_time[i], ground_time, ground_z.flatten()))
		g_int_roll[i,:] = (np.interp(slam_time[i], ground_time, ground_roll.flatten()))
		g_int_pitch[i,:] = (np.interp(slam_time[i], ground_time, ground_pitch.flatten()))
		g_int_yaw[i,:] = (np.interp(slam_time[i], ground_time, ground_yaw.flatten()))

		_g_int_matrix = tf_conversions.transformations.euler_matrix(g_int_roll[i], g_int_pitch[i], g_int_yaw[i], 'rxyz')
		_00 = _g_int_matrix[0,0]
		_01 = _g_int_matrix[0,1]
		_02 = _g_int_matrix[0,2]

		_g_int_matrix[0,3] = g_int_x[i]
		_03 = _g_int_matrix[0,3]

		_10 = _g_int_matrix[1,0]
		_11 = _g_int_matrix[1,1]
		_12 = _g_int_matrix[1,2]
		
		_g_int_matrix[1,3] = g_int_y[i]
		_13 = _g_int_matrix[1,3]

		_20 = _g_int_matrix[2,0]
		_21 = _g_int_matrix[2,1]
		_22 = _g_int_matrix[2,2]
		
		_g_int_matrix[2,3] = g_int_z[i]
		_23 = _g_int_matrix[2,3]

		_r_kitti = np.array([[0,-1,0,0],[0,0,-1,0],[1,0,0,0],[0,0,0,1]])
		_kitti_frame = np.matmul(_r_kitti, _g_int_matrix)
		_r00 = _kitti_frame[0,0]
		_r01 = _kitti_frame[0,1]
		_r02 = _kitti_frame[0,2]
		_r03 = _kitti_frame[0,3]
		_r10 = _kitti_frame[1,0]
		_r11 = _kitti_frame[1,1]
		_r12 = _kitti_frame[1,2]
		_r13 = _kitti_frame[1,3]
		_r20 = _kitti_frame[2,0]
		_r21 = _kitti_frame[2,1]
		_r22 = _kitti_frame[2,2]
		_r23 = _kitti_frame[2,3]

		output_data = [str(_r00), str(_r01), str(_r02), str(_r03), str(_r10), str(_r11), str(_r12), str(_r13), str(_r20), str(_r21), str(_r22), str(_r23)]     
		the_file.write(" ".join(output_data) + '\n')

	the_file.close()

	error_window_1 = 5 # number of frames to calculate error over
	x_error_window_1 = np.zeros((len(slam_input),1))
	y_error_window_1 = np.zeros((len(slam_input),1))
	z_error_window_1 = np.zeros((len(slam_input),1))
	roll_error_window_1 = np.zeros((len(slam_input),1))
	pitch_error_window_1 = np.zeros((len(slam_input),1))
	yaw_error_window_1 = np.zeros((len(slam_input),1))

	for i in range(len(slam_input)):
		if i < (len(slam_input) - error_window_1):
			_x_dist_traveled_slam = slam_x[i+error_window_1] - slam_x[i]
			_x_dist_traveled_gt = g_int_x[i+error_window_1] - g_int_x[i]
			x_error_window_1[i,:] = _x_dist_traveled_slam - _x_dist_traveled_gt

			_y_dist_traveled_slam = slam_y[i+error_window_1] - slam_y[i]
			_y_dist_traveled_gt = g_int_y[i+error_window_1] - g_int_y[i]
			y_error_window_1[i,:] = _y_dist_traveled_slam - _y_dist_traveled_gt

			_z_dist_traveled_slam = slam_z[i+error_window_1] - slam_z[i]
			_z_dist_traveled_gt = g_int_z[i+error_window_1] - g_int_z[i]
			z_error_window_1[i,:] = _z_dist_traveled_slam - _z_dist_traveled_gt

			_roll_diff_slam = slam_roll[i+error_window_1] - slam_roll[i]
			_roll_diff_gt = g_int_roll[i+error_window_1] - g_int_roll[i]
			roll_error_window_1[i,:] = _roll_diff_slam - _roll_diff_gt

			_pitch_diff_slam = slam_pitch[i+error_window_1] - slam_pitch[i]
			_pitch_diff_gt = g_int_pitch[i+error_window_1] - g_int_pitch[i]
			pitch_error_window_1[i,:] = _pitch_diff_slam - _pitch_diff_gt

			_yaw_diff_slam = slam_yaw[i+error_window_1] - slam_yaw[i]
			_yaw_diff_gt = g_int_yaw[i+error_window_1] - g_int_yaw[i]
			yaw_error_window_1[i,:] = _yaw_diff_slam - _yaw_diff_gt

	error_window_2 = 15 # number of frames to calculate error over
	x_error_window_2 = np.zeros((len(slam_input),1))
	y_error_window_2 = np.zeros((len(slam_input),1))
	z_error_window_2 = np.zeros((len(slam_input),1))
	roll_error_window_2 = np.zeros((len(slam_input),1))
	pitch_error_window_2 = np.zeros((len(slam_input),1))
	yaw_error_window_2 = np.zeros((len(slam_input),1))

	for i in range(len(slam_input)):
		if i < (len(slam_input) - error_window_2):
			_x_dist_traveled_slam = slam_x[i+error_window_2] - slam_x[i]
			_x_dist_traveled_gt = g_int_x[i+error_window_2] - g_int_x[i]
			x_error_window_2[i,:] = _x_dist_traveled_slam - _x_dist_traveled_gt

			_y_dist_traveled_slam = slam_y[i+error_window_2] - slam_y[i]
			_y_dist_traveled_gt = g_int_y[i+error_window_2] - g_int_y[i]
			y_error_window_2[i,:] = _y_dist_traveled_slam - _y_dist_traveled_gt

			_z_dist_traveled_slam = slam_z[i+error_window_2] - slam_z[i]
			_z_dist_traveled_gt = g_int_z[i+error_window_2] - g_int_z[i]
			z_error_window_2[i,:] = _z_dist_traveled_slam - _z_dist_traveled_gt

			_roll_diff_slam = slam_roll[i+error_window_2] - slam_roll[i]
			_roll_diff_gt = g_int_roll[i+error_window_2] - g_int_roll[i]
			roll_error_window_2[i,:] = _roll_diff_slam - _roll_diff_gt

			_pitch_diff_slam = slam_pitch[i+error_window_2] - slam_pitch[i]
			_pitch_diff_gt = g_int_pitch[i+error_window_2] - g_int_pitch[i]
			pitch_error_window_2[i,:] = _pitch_diff_slam - _pitch_diff_gt

			_yaw_diff_slam = slam_yaw[i+error_window_2] - slam_yaw[i]
			_yaw_diff_gt = g_int_yaw[i+error_window_2] - g_int_yaw[i]
			yaw_error_window_2[i,:] = _yaw_diff_slam - _yaw_diff_gt


	error_window_3 = 100 # number of frames to calculate error over
	x_error_window_3 = np.zeros((len(slam_input),1))
	y_error_window_3 = np.zeros((len(slam_input),1))
	z_error_window_3 = np.zeros((len(slam_input),1))
	roll_error_window_3 = np.zeros((len(slam_input),1))
	pitch_error_window_3 = np.zeros((len(slam_input),1))
	yaw_error_window_3 = np.zeros((len(slam_input),1))

	for i in range(len(slam_input)):
		if i < (len(slam_input) - error_window_3):
			_x_dist_traveled_slam = slam_x[i+error_window_3] - slam_x[i]
			_x_dist_traveled_gt = g_int_x[i+error_window_3] - g_int_x[i]
			x_error_window_3[i,:] = _x_dist_traveled_slam - _x_dist_traveled_gt

			_y_dist_traveled_slam = slam_y[i+error_window_3] - slam_y[i]
			_y_dist_traveled_gt = g_int_y[i+error_window_3] - g_int_y[i]
			y_error_window_3[i,:] = _y_dist_traveled_slam - _y_dist_traveled_gt

			_z_dist_traveled_slam = slam_z[i+error_window_3] - slam_z[i]
			_z_dist_traveled_gt = g_int_z[i+error_window_3] - g_int_z[i]
			z_error_window_3[i,:] = _z_dist_traveled_slam - _z_dist_traveled_gt

			_roll_diff_slam = slam_roll[i+error_window_3] - slam_roll[i]
			_roll_diff_gt = g_int_roll[i+error_window_3] - g_int_roll[i]
			roll_error_window_3[i,:] = _roll_diff_slam - _roll_diff_gt

			_pitch_diff_slam = slam_pitch[i+error_window_3] - slam_pitch[i]
			_pitch_diff_gt = g_int_pitch[i+error_window_3] - g_int_pitch[i]
			pitch_error_window_3[i,:] = _pitch_diff_slam - _pitch_diff_gt

			_yaw_diff_slam = slam_yaw[i+error_window_3] - slam_yaw[i]
			_yaw_diff_gt = g_int_yaw[i+error_window_3] - g_int_yaw[i]
			yaw_error_window_3[i,:] = _yaw_diff_slam - _yaw_diff_gt

			# print('slam time: ', slam_time[i], ' i: ', i, ' x diff error: ', x_error_window_1[i])

	###
	### Gradients for debugging
	###

	slam_x_gradient = np.gradient(slam_x.flatten())
	gt_x_gradient = np.gradient(g_int_x.flatten())

	###
	### Plots ###
	###

	plt.figure(1)
	plt.subplot(211)
	plt.plot(slam_time, slam_x, label='slam x (m)')
	plt.plot(slam_time, g_int_x, label='ground x (m)')
	plt.legend()
	plt.subplot(212)
	plt.plot(slam_time, slam_y, label='slam y (m)')
	plt.plot(slam_time, g_int_y, label='ground y (m)')
	plt.legend()

	plt.figure(2)
	plt.subplot(211)
	plt.plot(slam_time, slam_roll, label='slam roll (rad)')
	plt.plot(slam_time, g_int_roll, label='ground roll (rad)')
	plt.plot(slam_time, slam_pitch, label='slam pitch (rad)')
	plt.plot(slam_time, g_int_pitch, label='ground pitch (rad)')
	plt.legend()
	plt.subplot(212)
	plt.plot(slam_time, slam_yaw, label='slam yaw (rad)')
	plt.plot(slam_time, g_int_yaw, label='ground yaw (rad)')
	plt.legend()

	plt.figure(3)
	plt.plot(slam_x, slam_y, label='slam')
	plt.plot(g_int_x, g_int_y, label='gt')
	plt.legend()
	
	plt.show()


if __name__ == '__main__':
	main()
