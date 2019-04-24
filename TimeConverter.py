#!/usr/bin/python

import argparse
import numpy as np
import datetime
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def parse_args():
	arg_parser = argparse.ArgumentParser(description='convert vectornav gps time to posix/utc')
	arg_parser.add_argument('--input_file', 
							required=True, 
							help='vectornav CSV input file')
	arg_parser.add_argument('--output_file',
							required=True,
							help='converted vectornav CSV output file with UTC time')
	return arg_parser.parse_args()

# gpsTime = 1240083069.365325312

def gpsTimeToPosix(gpsTime, leapSeconds=18):
	gpsOffset = (datetime.datetime(1980, 1, 6) - datetime.datetime(1970, 1, 1)).total_seconds()
	gpsTimestamp = gpsTime + gpsOffset - leapSeconds
	return gpsTimestamp
	
	# print("original gps time:")
	# print(gpsTime)
	# print("gps to UTC offset")
	# print(gpsOffset)
	# print("new gps time:")
	# print(gpsTimestamp)

def main():
	args = parse_args()
	epochUTC = datetime.datetime(1970,1,1,0,0,0)
	nowUTC = datetime.datetime.utcnow()
	timestampUTC = (nowUTC - epochUTC).total_seconds()
	
	print("current utc time, fyi: ")
	print(timestampUTC)
	
	# print("previous gps time: ")
	# print(gpsTime)
	# print("new utc/posix time: ")
	# print(gpsTimeToPosix(gpsTime, leapSeconds=18))
	
	inputData = np.genfromtxt(args.input_file, skip_header=1, delimiter=',')
	print("data has been imported")

	gpsUTCPosix = np.zeros((len(inputData),1))
	accX = inputData[:,1]
	accY = inputData[:,2]
	accZ = inputData[:,3]
	gyroX = inputData[:,4]
	gyroY = inputData[:,5]
	gyroZ = inputData[:,6]
	lat = inputData[:,14]
	longitude = inputData[:,15]
	alt = inputData[:,16]
	yaw = inputData[:,17]
	pitch = inputData[:,18]
	roll = inputData[:,19]
	quat0 = inputData[:,20]
	quat1 = inputData[:,21]
	quat2 = inputData[:,22]
	quatW = inputData[:,23]

	# TimeGps,AccX,AccY,AccZ,GyroX,GyroY,GyroZ,Year,Month,Day,Hour,Minute,Second,FracSec,Lat,Lon,Alt,Yaw,Pitch,Roll,Quat[0],Quat[1],Quat[2],Quat[3],VelX,VelY,VelZ
	for i, row in enumerate(inputData):
		gpsTime = row[0]/1000000000 # microsecond -> second
		print("previous gps time:")
		print(gpsTime)
		print("new utc/posix time:")
		print(gpsTimeToPosix(gpsTime, leapSeconds=18))
		gpsUTCPosix[i,:	] = (gpsTimeToPosix(gpsTime, 18))

	dataOutput = np.column_stack((gpsUTCPosix,accX,accY,accZ,gyroX,gyroY,gyroZ,lat,longitude,alt,yaw,pitch,roll,quat0,quat1,quat2,quatW))
	np.savetxt(args.output_file, dataOutput, fmt='%.8f', delimiter=' ', header="### gpsUTCPosix,accX,accY,accZ,gyroX,gyroY,gyroZ,lat,longitude,alt,yaw,pitch,roll,quat0,quat1,quat2,quatW", comments='')

	plt.subplot(4,1,1)
	plt.plot(gpsUTCPosix, pitch, '-o', color='gold')
	plt.ylabel('pitch (deg)')
	plt.title('attitude')

	plt.subplot(4,1,2)
	plt.plot(gpsUTCPosix, roll, '-o', color='green')
	plt.ylabel('roll (deg)')

	plt.subplot(4,1,3)
	plt.plot(gpsUTCPosix, yaw, '-o', color='red')
	plt.ylabel('yaw (deg)')
	plt.xlabel('utc/posix time (s)')

	plt.subplot(4,1,4)
	plt.plot(lat, longitude, '-o', color='darkcyan')
	plt.ylabel('longitude (deg)')
	plt.xlabel('latitude (deg)')

	plt.show()

if __name__=='__main__':
	main()
