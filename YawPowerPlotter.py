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
	
	data = np.genfromtxt(args.input, skip_header=16, skip_footer=100)
	print("data has been imported")	

	dataRows = np.size(data,0)
	print("num rows: ", dataRows)

	# accel group
	sysTime = data[:,0]
	vnAccX = data[:,2]
	izzeAccX = data[:,3]
	vnAccY = data[:, 4]
	izzeAccY = data[:, 5]
	vnAccZ = data[:, 6]
	izzeAccZ = data[:, 7]	
	
	# position group
	vnLat = data[:,9]
	vnLong = data[:,10]

	# orientation group
	vnPitch = data[:, 11]*(180/np.pi)
	vnRoll = data[:, 12]*(180/np.pi)
	vnYaw = data[:, 13]*(180/np.pi)

	#eTesta alt (garbage data)
	etAlt = data[:, 14]

	# engine data
	rpm = data[:, 26]

	# global vel
	vnVelGlobalX = data[:, 34]
	vnVelGlobalY = data[:, 35]
	vnVelGlobalZ = data[:, 36]

	vnRelVelX = np.zeros((len(data),1))
	vnRelVelY = np.zeros((len(data),1))
	vnSpd = np.zeros((len(data),1))
	zero = np.zeros((len(data), 1))


	# for i, row in enumerate(vnData):
	# 	vnTime = row[0]
	# 	# print(vnTime)
	# 	vnTimeOut[i,:] = vnTime		

	print(sysTime)

	for i, row in enumerate(data):
		forVnYaw = row[13]
		forVnVelX = row[34]
		forVnVelY = row[35]
		forVnVelZ = row[36]
		vnRelVelX[i,:] = (np.sin(forVnYaw)*forVnVelY + np.cos(forVnYaw)*forVnVelX)
		# print(vnRelVelX)
		vnRelVelY[i,:] = (np.cos(forVnYaw)*forVnVelY + (-np.sin(forVnYaw)*forVnVelX))
		# print(vnRelVelY)
		vnSpd[i,:] = np.sqrt((forVnVelX**2)+(forVnVelY**2)+(forVnVelZ**2))
		zero[i,:] = 0

	print(vnSpd)

	fig, (ax1, ax2) = plt.subplots(2,1, sharex=True)
	fig.suptitle('lat long (dd)')
	ax1.plot(sysTime, vnLat, label='latitude (dd)')
	ax2.plot(sysTime, vnLong, label='longitude (dd)')
	ax1.legend()
	ax2.legend()

	fig2, (ax3, ax4) = plt.subplots(2,1, sharex=True)
	fig2.suptitle('orientation')
	ax3.plot(sysTime, vnRoll, color='red', label='roll')
	ax3.plot(sysTime, vnPitch, color='green', label='pitch')
	ax3.legend()

	ax4.plot(sysTime, vnRelVelX, '-o', color='red', label='rel vel x')
	ax4.plot(sysTime, vnRelVelY, '-o', color='magenta', label='rel vel y')
	ax4.plot(sysTime, vnVelGlobalZ, color='blue', label='global vel z')
	# ax4.plot(vnSpd, color='green', label='speed (m/s)')
	ax4.plot(sysTime, zero, color='black')
	ax4.legend()

	# plt.figure(2)
	# plt.subplot(311)
	# plt.title('roll and pitch (deg)')
	# plt.plot(sysTime, vnRoll, color='red', label='roll')
	# plt.plot(sysTime, vnPitch, color='green', label='pitch')
	# plt.legend()
	# plt.ylim(-20,20)
	# plt.xlim(2400,2500)

	# plt.subplot(312)
	# plt.title('body velocity (m/s)')
	# plt.plot(sysTime, vnRelVelX, color='red', label='rel vel x')
	# plt.plot(sysTime, vnRelVelY, color='magenta', label='rel vel y')
	# plt.plot(sysTime, vnVelGlobalZ, color='blue', label='global vel z')
	# plt.plot(sysTime, vnSpd, color='green', label='speed (m/s)')	
	# plt.plot(sysTime, zero, color='black')
	# plt.legend()
	# plt.xlim(2400,2500)

	# plt.subplot(313)
	# plt.title('global (NED) velocity (m/s)')
	# plt.plot(sysTime, vnVelGlobalX, color='pink', label='global vel x')
	# plt.plot(sysTime, vnVelGlobalY, color='blue', label='global vel y')
	# plt.plot(sysTime, vnVelGlobalZ, color='green', label='global vel z')
	# plt.legend()
	# plt.xlim(2450,2500)

	plt.show()

if __name__=='__main__':
	main()
