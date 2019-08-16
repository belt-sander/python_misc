#!/usr/bin/python

import argparse
import numpy as np
import datetime
import matplotlib.pyplot as plt

def parse_args():
	arg_parser = argparse.ArgumentParser(description='data plotting tools for evaluation of data from paul yaw')
	arg_parser.add_argument('-i',
							'--input', 
							required=True, 
							help='text file output from WinDarab')
	return arg_parser.parse_args()

def main():
	args = parse_args()
	
	data = np.genfromtxt(args.input, skip_header=6, delimiter='	')
	print("data has been imported")	

	dataRows = np.size(data,0)
	print("num rows: ", dataRows)

	# accel group
	sysTime = data[:,0]
	vnAccX = data[:,1]
	izzeAccX = data[:,2]
	vnAccY = data[:, 3]
	izzeAccY = data[:, 4]
	vnAccZ = data[:, 5]
	izzeAccZ = data[:, 6]	
	
	# position group
	vnLat = data[:,8]
	vnLong = data[:,9]

	# orientation group
	vnPitch = data[:, 10]
	vnRoll = data[:, 11]
	vnYaw = data[:, 12]

	#eTesta alt (garbage data)
	etAlt = data[:, 13]

	# engine data
	rpm = data[:, 30]

	# global vel
	vnVelGlobalX = data[:, 40]
	vnVelGlobalY = data[:, 41]
	vnVelGlobalZ = data[:, 42]

	vnRelVelX = np.zeros((len(data),1))
	vnRelVelY = np.zeros((len(data),1))
	vnSpd = np.zeros((len(data),1))
	zero = np.zeros((len(data), 1))


	# for i, row in enumerate(vnData):
	# 	vnTime = row[0]
	# 	# print(vnTime)
	# 	vnTimeOut[i,:] = vnTime		

	for i, row in enumerate(data):
		forVnYaw = row[12]
		forVnVelX = row[40]
		forVnVelY = row[41]
		forVnVelZ = row[42]
		vnRelVelX[i,:] = (np.sin(forVnYaw)*forVnVelY + np.cos(forVnYaw)*forVnVelX)
		# print(vnRelVelX)
		vnRelVelY[i,:] = (np.cos(forVnYaw)*forVnVelY + (-np.sin(forVnYaw)*forVnVelX))
		# print(vnRelVelY)
		# vnSpd[i,:] = np.sqrt((forVnVelX^2)+(forVnVelY^2)+(forVnVelZ^2))
		# print(vnSpd)
		zero[i,:] = 0

	plt.figure(1)	
	plt.subplot(411)
	plt.title('lat / long (dd)')
	plt.plot(vnLat, vnLong, color='blue')

	plt.subplot(412)
	plt.title('roll and pitch (rad)')
	plt.plot(sysTime, vnRoll, color='red', label='roll')
	plt.plot(sysTime, vnPitch, color='green', label='pitch')
	plt.legend()

	plt.subplot(413)
	plt.title('body velocity (m/s)')
	plt.plot(sysTime, vnRelVelX, color='red', label='rel vel x')
	plt.plot(sysTime, vnRelVelY, color='magenta', label='rel vel y')
	plt.plot(sysTime, vnVelGlobalZ, color='blue', label='global vel z')
	plt.plot(sysTime, zero, color='black')
	plt.legend()

	plt.subplot(414)
	plt.title('global (NED) velocity (m/s)')
	plt.plot(sysTime, vnVelGlobalX, color='pink', label='global vel x')
	plt.plot(sysTime, vnVelGlobalY, color='blue', label='global vel y')
	plt.plot(sysTime, vnVelGlobalZ, color='green', label='global vel z')
	plt.legend()

	plt.show()

if __name__=='__main__':
	main()
