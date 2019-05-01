#!/usr/bin/python

import argparse
import numpy as np
import datetime
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def parse_args():
	arg_parser = argparse.ArgumentParser(description='add or subtract Z height from trajectory file')
	arg_parser.add_argument('--input_file', 
							required=True, 
							help='trajectory input file')
	arg_parser.add_argument('--output_file',
							required=True,
							help='offset trajectory output file name')
	arg_parser.add_argument('--height_offset',
							required=True,
							type=int,
							help='amount to offset height above the ellipsoid')
	return arg_parser.parse_args()

def main():
	args = parse_args()
	
	inputData = np.genfromtxt(args.input_file, skip_header=30, skip_footer=30, delimiter='')
	print("")
	print("data has been imported")
	print("")

	new_hMSL = np.zeros((len(inputData),1))
	old_hMSL = np.zeros((len(inputData),1))
    # hEll = inputData[:,14]	

	accX = inputData[:,0]
	accY = inputData[:,1]
	accZ = inputData[:,2]
	gyroX = inputData[:,3]
	gyroY = inputData[:,4]
	gyroZ = inputData[:,5]
	yaw = inputData[:,6]
	pitch = inputData[:,7]
	roll = inputData[:,8]
	utmX = inputData[:,9]
	utmY = inputData[:,10]
	hEll = inputData[:,11]
	utc = inputData[:,12]
	und = inputData[:,13]

   	for i, row in enumerate(inputData):
		hMSL = row[14]
		new_hMSL[i,: ] = hMSL+(args.height_offset)
		old_hMSL[i,: ] = hMSL

	dataOutput = np.column_stack((accX,accY,accZ,gyroX,gyroY,gyroZ,yaw,pitch,roll,utmX,utmY,new_hMSL,utc,und,hEll))
	np.savetxt(args.output_file, dataOutput, fmt='%.8f', delimiter=' ', header="# accX,accY,accZ,gyroX,gyroY,gyroZ,yaw,pitch,roll,utmX,utmY,hEll", comments='')
	print("data has been exported")
	print("")

	plt.subplot(5,1,1)
	plt.plot(utc, pitch, '-o', color='gold')
	plt.ylabel('pitch (deg)')
	plt.title('attitude')

	plt.subplot(5,1,2)
	plt.plot(utc, roll, '-o', color='green')
	plt.ylabel('roll (deg)')

	plt.subplot(5,1,3)
	plt.plot(utc, yaw, '-o', color='red')
	plt.ylabel('yaw (deg)')
	plt.xlabel('utc/posix time (s)')

	plt.subplot(5,1,4)
	plt.plot(utmX, utmY, '-o', color='darkcyan')
	plt.ylabel('longitude (deg)')
	plt.xlabel('latitude (deg)')

	plt.subplot(5,1,5)
	plt.plot(utc, old_hMSL)
	plt.plot(utc, new_hMSL)
	plt.xlabel('utc/posix time (s)')
	plt.ylabel('height mean sea level')

	plt.show()

if __name__=='__main__':
	main()
