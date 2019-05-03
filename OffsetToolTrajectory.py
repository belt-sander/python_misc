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
	arg_parser.add_argument('--x_offset',
							required=True,
							type=int,
							help='offset to x / easting UTM')
	arg_parser.add_argument('--y_offset',
							required=True,
							type=int,
							help='offset to y / northing UTM')
	return arg_parser.parse_args()

def main():
	args = parse_args()
	
	inputData = np.genfromtxt(args.input_file, skip_header=0, skip_footer=0, delimiter='')
	print("")
	print("data has been imported")
	print("")

	new_height = np.zeros((len(inputData),1))
	old_height = np.zeros((len(inputData),1))

	new_utmX = np.zeros((len(inputData),1))
	old_utmX = np.zeros((len(inputData),1))

	new_utmY = np.zeros((len(inputData),1))
	old_utmY = np.zeros((len(inputData),1))		

	matrix1 = inputData[:,0]
	matrix2 = inputData[:,1]
	matrix3 = inputData[:,2]
	matrix4 = inputData[:,3]
	matrix5 = inputData[:,4]
	matrix6 = inputData[:,5]
	matrix7 = inputData[:,6]
	matrix8 = inputData[:,7]
	matrix9 = inputData[:,8]
	# utmX = inputData[:,9]
	# utmY = inputData[:,10]
	# height = inputData[:,11]
	utc = inputData[:,12]

   	for i, row in enumerate(inputData):
		height = row[11]
		new_height[i,: ] = height+(args.height_offset)
		old_height[i,: ] = height
		utmX = row[9]
		new_utmX[i,: ] = utmX+(args.x_offset)
		old_utmX[i,: ] = utmX
		utmY = row[10]
		new_utmY[i,: ] = utmY+(args.y_offset)
		old_utmY[i,: ] = utmY

	dataOutput = np.column_stack((matrix1,matrix2,matrix3,matrix4,matrix5,matrix6,matrix7,matrix8,matrix9,new_utmX,new_utmY,new_height,utc))
	np.savetxt(args.output_file, dataOutput, fmt='%.8f', delimiter=' ', header="# 3x3 rotation matrix,utmX,utmY,height", comments='')
	print("data has been exported")
	print("")

	plt.subplot(2,1,1)
	plt.plot(new_utmX, new_utmY, '-o', color='darkcyan')
	plt.ylabel('longitude (deg)')
	plt.xlabel('latitude (deg)')

	plt.subplot(2,1,2)
	plt.plot(utc, old_height)
	plt.plot(utc, new_height)
	plt.xlabel('utc/posix time (s)')
	plt.ylabel('height mean sea level')

	plt.show()

if __name__=='__main__':
	main()
