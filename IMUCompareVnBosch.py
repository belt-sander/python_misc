#!/usr/bin/python

from __future__ import print_function
import argparse
import numpy as np
import datetime
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def parse_args():
	arg_parser = argparse.ArgumentParser(description='convert raw IMU data capture from pcb-tools to real data values')
	arg_parser.add_argument('-i',
							'--bosch_file', 
							required=True, 
							help='imu data input file from output of IMUDataConverter.py')
	arg_parser.add_argument('-m',
							'--motec_file',
							required=True,
							help='imu data input file from output of MoTeCGPSTimeConvert.py')
	return arg_parser.parse_args()

def main():
	args = parse_args()
	
	boschData = np.genfromtxt(args.bosch_file, skip_header=2)
	compareData = np.genfromtxt(args.motec_file, skip_header=2)

	print("")
	print("data has been imported")
	print("")

	### motec is accel xyz gyro xyz columns 15-20 ###
	### bosch is gyro xyz accel xyz columns 1-6 ###

	bosch_accel_x = boschData[:,4]
	bosch_accel_y = boschData[:,5]
	bosch_accel_z = boschData[:,6]
	bosch_gyro_x = boschData[:,1]
	bosch_gyro_y = boschData[:,2]
	bosch_gyro_z = boschData[:,3]
	bosch_time = boschData[:,0]
	vn_accel_x = compareData[:,15]
	vn_accel_y = compareData[:,16]
	vn_accel_z = compareData[:,17]
	vn_gyro_x = compareData[:,18]
	vn_gyro_y = compareData[:,19]
	vn_gyro_z = compareData[:,20]
	vn_time = compareData[:,0]

	interp_vn_gyro_z = (np.interp(bosch_time, vn_time, vn_gyro_z))*(-1) # sign flip due to reference frame differences
	interp_vn_gyro_y = (np.interp(bosch_time, vn_time, vn_gyro_y))*(-1)
	interp_vn_gyro_x = (np.interp(bosch_time, vn_time, vn_gyro_x))*(-1)
	interp_vn_accel_x = (np.interp(bosch_time, vn_time, vn_accel_x))/9.81 # convert to G from m/s/s
	interp_vn_accel_y = (np.interp(bosch_time, vn_time, vn_accel_y))/9.81
	interp_vn_accel_z = (np.interp(bosch_time, vn_time, vn_accel_z))/9.81 * -1 # sign flip


	print('bosch time: ', bosch_time)
	print('')
	print('vn time: ', vn_time)
	print('')

	plt.figure(1)
	plt.subplot(311)
	plt.plot(bosch_time, interp_vn_gyro_z, label="vn gyro z")
	plt.plot(bosch_time, bosch_gyro_z, label="bosch gyro z")
	plt.legend()
	plt.title("imu time sync check")
	# plt.xlim(1.570141e9+580,1.570141e9+660) ### south bound highway time region
	# plt.xlim(1.570142e9-100, 1.570142e9+5) ### north bound highway time region

	plt.subplot(312)
	plt.plot(bosch_time, interp_vn_gyro_y, label="vn gyro y")
	plt.plot(bosch_time, bosch_gyro_y, label="bosch gyro y")
	plt.legend()
	# plt.xlim(1.570141e9+580,1.570141e9+660) ### south bound highway time region
	# plt.xlim(1.570142e9-100, 1.570142e9+5) ### north bound highway time region

	plt.subplot(313)
	plt.plot(bosch_time, interp_vn_gyro_x, label="vn gyro x")
	plt.plot(bosch_time, bosch_gyro_x, label="bosch gyro x")
	plt.legend()
	# plt.xlim(1.570141e9+580,1.570141e9+660) ### south bound highway time region
	# plt.xlim(1.570142e9-100, 1.570142e9+5) ### north bound highway time region

	plt.figure(2)
	plt.subplot(311)
	plt.plot(bosch_time, interp_vn_accel_z, label="vn_accel_z (G)")
	plt.plot(bosch_time, bosch_accel_z, label="bosch_accel_z (G)")
	# plt.xlim(1.570141e9+580,1.570141e9+660) ### south bound highway time region
	# plt.xlim(1.570142e9-100, 1.570142e9+5) ### north bound highway time region
	plt.legend()
	plt.title('imu time sync check accel Z')

	plt.subplot(312)
	plt.plot(bosch_time, interp_vn_accel_y, label="vn_accel_y (G)")
	plt.plot(bosch_time, bosch_accel_x, label="bosch_accel_x (G)")
	# plt.xlim(1.570141e9+580,1.570141e9+660) ### south bound highway time region
	# plt.xlim(1.570142e9-100, 1.570142e9+5) ### north bound highway time region
	plt.legend()

	plt.subplot(313)
	plt.plot(bosch_time, interp_vn_accel_x, label="vn_accel_x (G)")
	plt.plot(bosch_time, bosch_accel_y, label="bosch_accel_y (G)")
	# plt.xlim(1.570141e9+580,1.570141e9+660) ### south bound highway time region
	# plt.xlim(1.570142e9-100, 1.570142e9+5) ### north bound highway time region
	plt.legend()




	plt.show()

if __name__=='__main__':
	main()
