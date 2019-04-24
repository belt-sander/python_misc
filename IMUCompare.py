#!/usr/bin/python

import argparse
import numpy as np
import datetime
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def parse_args():
	arg_parser = argparse.ArgumentParser(description='plot and compare imu data between ubx and vn')
	arg_parser.add_argument('--vn_input_file', 
							required=True, 
							help='vectornav CSV input file')
	arg_parser.add_argument('--ubx_input_file_acc',
							required=True,
							help='ubx CSV input accel/gyro file')
	arg_parser.add_argument('--ubx_input_file_orient',
							required=True,
							help='ubx CSV input orientation file')
	return arg_parser.parse_args()

def main():
	args = parse_args()
	
	vnData = np.genfromtxt(args.vn_input_file, skip_header=1)
	ubxData = np.genfromtxt(args.ubx_input_file_acc, skip_header=8, delimiter=',')
	ubxOrientData = np.genfromtxt(args.ubx_input_file_orient, skip_header=6, delimiter=',')
	print("data has been imported")

	num_vn_poses = np.size(vnData,0)
	num_ubx_poses = np.size(ubxData,0)
	num_ubx_orient_poses = np.size(ubxOrientData,0)
	print("num vn poses: ", num_vn_poses)
	print("num ubx poses: ", num_ubx_poses)
	print("num ubx orient poses:", num_ubx_orient_poses)

	vnTimeOut = np.zeros((len(vnData),1))
	ubxTimeOut = np.zeros((len(ubxData), 1))
	ubxOrientTimeOut = np.zeros((len(ubxData), 1))

	for i, row in enumerate(vnData):
		vnTime = row[0]
		# print(vnTime)
		vnTimeOut[i,:] = vnTime
		
	for i, row in enumerate(ubxData):
		ubxTime = row[0]
		# print(ubxTime)
		ubxTimeOut[i,:] = ubxTime
	
	for i, row in enumerate(ubxOrientData):
		ubxOrientTime = row[0]
		# print(ubxOrientTime)
		ubxOrientTimeOut[i,:] = ubxOrientTime

	print(vnTimeOut)
	print(ubxTimeOut)
	print(ubxOrientTimeOut)

	# ap method, ugly af
	# deltaT = np.zeros((num_ubx_poses, num_vn_poses))
	# for ii in range(num_ubx_poses):
	# 	for jj in range(num_vn_poses):
	# 		deltaT[ii,jj] = abs(ubxData[ii,0] - vnData[jj,0])
	# errors = []

	# returns exact matches only
	# n = min(len(vnTimeOut), len(ubxTimeOut))
	# out_idx = np.flatnonzero(vnTimeOut[:n] == ubxTimeOut[:n])
	# out_val = vnTimeOut[out_idx]
	# print(out_val)

	accX = vnData[:,1]
	accY = vnData[:,2]
	accZ = vnData[:,3]
	gyroX = vnData[:,4]
	gyroY = vnData[:,5]
	gyroZ = vnData[:,6]
	lat = vnData[:,7]
	longitude = vnData[:,8]
	alt = vnData[:,9]
	yaw = vnData[:,10]
	pitch = vnData[:,11]
	roll = vnData[:,12]
	quat0 = vnData[:,13]
	quat1 = vnData[:,14]
	quat2 = vnData[:,15]
	quatW = vnData[:,16]

	ubxAccX = ubxData[:,1]
	ubxAccY = ubxData[:,2]
	ubxAccZ = ubxData[:,3]
	ubxGyroX = ubxData[:,4]
	ubxGyroY = ubxData[:,5]
	ubxGyroZ = ubxData[:,6]

	ubxPreOrientTime = ubxOrientData[:,0]
	ubxAttitudeX = ubxOrientData[:,1] / 1e5
	ubxAttitudeY = ubxOrientData[:,2] / 1e5
	ubxAttitudeZ = ubxOrientData[:,3] / 1e5

	plt.figure(1)
	plt.subplot(3,1,1)
	plt.title('attitude')
	plt.plot(vnTimeOut, roll, '-o', color='gold')
	plt.plot(vnTimeOut, pitch, '-o', color='green')
	# plt.plot(vnTimeOut, yaw, '-o', color='red')
	plt.xlim((-20+1.55606216e9), (60+1.55606216e9))
	plt.ylim(-10,14)
	plt.subplot(3,1,2)
	plt.title('ubx attitude')
	plt.plot(ubxPreOrientTime, ubxAttitudeX, '-o', color='gold')
	plt.plot(ubxPreOrientTime, ubxAttitudeY, '-o', color='green')
	# plt.plot(ubxPreOrientTime, ubxAttitudeZ, '-o', color='red')
	plt.xlim((-20+1.55606216e9), (60+1.55606216e9))
	plt.ylim(-10,14)
	plt.xlabel('posix UTC (s)')
	
	plt.subplot(3,1,3)
	plt.plot(vnTimeOut, longitude, '-o', color='darkcyan')
	# plt.plot(vnTimeOut, lat, '-o', color='blue')
	plt.ylabel('lat / long (deg)')
	plt.xlim((-20+1.55606216e9), (60+1.55606216e9))
	plt.xlabel('posix UTC (s)')

	plt.figure(2)
	plt.subplot(2,1,1)
	plt.plot(vnTimeOut, accX, '-o', color='blue')
	plt.plot(vnTimeOut, accY, '-o', color='red')
	plt.plot(vnTimeOut, accZ, '-o', color='green')
	plt.ylabel('m/s/s')
	plt.xlabel('posix UTC (s)')
	plt.xlim((6.8+1.55606216e9), (7.8+1.55606216e9))
	plt.ylim(-11,3)
	plt.title('vectornav acceleration data')
	plt.subplot(2,1,2)
	plt.plot(ubxTimeOut, ubxAccX, '-o', color='blue')
	plt.plot(ubxTimeOut, ubxAccY, '-o', color='red')
	plt.plot(ubxTimeOut, ubxAccZ, '-o', color='green')
	plt.ylabel('m/s/s')
	plt.xlabel('posix UTC (s)')
	plt.xlim((6.8+1.55606216e9), (7.8+1.55606216e9))
	plt.title('ubx accleration data')
	plt.ylim(-11,3)

	plt.show()

if __name__=='__main__':
	main()
