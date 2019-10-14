#!/usr/bin/python

import argparse
import numpy as np
import datetime
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def parse_args():
	arg_parser = argparse.ArgumentParser(description='convert raw IMU data capture from pcb-tools to real data values')
	arg_parser.add_argument('-i',
							'--input_file', 
							required=True, 
							help='imu data input file')
	arg_parser.add_argument('-o',
							'--output_file',
							required=True,
							help='imu data output file')
	return arg_parser.parse_args()

def main():
	args = parse_args()
	
	inputData = np.genfromtxt(args.input_file, skip_header=2, delimiter=',')
	print("")
	print("data has been imported")
	print("")

	converted_accel_x = np.zeros((len(inputData),1))
	converted_accel_y = np.zeros((len(inputData),1))
	converted_accel_z = np.zeros((len(inputData),1))
	converted_gyro_x = np.zeros((len(inputData),1))
	converted_gyro_y = np.zeros((len(inputData),1))
	converted_gyro_z = np.zeros((len(inputData),1))
	converted_time = np.zeros((len(inputData),1))

	sysTime = inputData[:,0]

   	for i, row in enumerate(inputData):
		gyroX = row[4]
		gyroY = row[5]
		gyroZ = row[6]
		accelX = row[7]
		accelY = row[8]
		accelZ = row[9]
		gpsTime = row[1]

		converted_accel_x[i,: ] = (accelX / 32768)*2
		converted_accel_y[i,: ] = (accelY / 32768)*2
		converted_accel_z[i,: ] = (accelZ / 32768)*2
		converted_gyro_x[i,: ] = (gyroX / 32768) * 2000
		converted_gyro_y[i,: ] = (gyroY / 32768) * 2000
		converted_gyro_z[i,: ] = (gyroZ / 32768) * 2000				
		converted_time[i,: ] = gpsTime / 1e6 # uS to Seconds

	dataOutput = np.column_stack((converted_time, converted_gyro_x, converted_gyro_y, converted_gyro_z, converted_accel_x, converted_accel_y, converted_accel_z))
	np.savetxt(args.output_file, dataOutput, fmt='%.8f', delimiter=' ', header="# utcTime (sec), gyro X (deg/sec), gyro Y (deg/sec), gyro Z (deg/sec), accel X (G), accel Y (G), accel Z (G)", comments='')
	print("data has been exported")
	print("")

	plt.figure(1)
	plt.subplot(211)
	plt.plot(converted_time, converted_accel_x, label="accel x (G)")
	plt.plot(converted_time, converted_accel_y, label="accel y (G)")
	plt.plot(converted_time, converted_accel_z, label="accel z (G)")
	plt.legend()

	plt.subplot(212)
	plt.plot(converted_time, converted_gyro_x, label="gyro x")
	plt.plot(converted_time, converted_gyro_y, label="gyro y")
	plt.plot(converted_time, converted_gyro_z, label="gyro z")
	plt.legend()

	plt.show()

if __name__=='__main__':
	main()
