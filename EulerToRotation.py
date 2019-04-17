#!/usr/bin/python
import numpy as np
import argparse
from transformations import euler_from_matrix

def parse_args():
	arg_parser = argparse.ArgumentParser(description='generate mapper trajectory file from novatel session_localization.txt file')
	arg_parser.add_argument('--input_file', 
							required=True, 
							help='novatel session_localization.txt file')
	arg_parser.add_argument('--output_file',
							required=True,
							help='resulting mapper trajectory output file')
	return arg_parser.parse_args()

def main():
	# data input
	args = parse_args()
	inputData = np.genfromtxt(args.input_file , skip_header=27, skip_footer=10)
	print("data has been imported")
	
	# bringing in data from input file
	easting = inputData[:,9]
	northing = inputData[:,10]
	alt = inputData[:,14]
	time = inputData[:,12]
	output = np.zeros((len(inputData),9), dtype=np.float32)

	# euler to rotation matrix conversion using novatel reference frame (y=forward, x=right, z=up)
	for i, row in enumerate(inputData):
		yaw = row[6]/180*np.pi
		pitch = row[7]/180*np.pi
		roll = row[8]/180*np.pi
		rotation_x = np.array([[1, 0, 0],
	                	   	  [0, np.cos(roll), -np.sin(roll)],
	                	   	  [0, np.sin(roll), np.cos(roll)]])

		rotation_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
		                	   [0, 1, 0],
		                	   [-np.sin(pitch), 0, np.cos(pitch)]])
	                 
		rotation_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
		                	   [np.sin(yaw), np.cos(yaw), 0],
		                	   [0, 0, 1]])

		rotationMatrix = np.dot(rotation_z, np.dot(rotation_y, rotation_x ))
		output[i,:] = rotationMatrix.reshape((1, 9))

		print("my matrix: ")
		print(rotationMatrix)

		# standard euler angle representation: roll, pitch, yaw
		print("novatel euler: (roll, pitch, yaw)")
		print(roll, pitch, yaw)

		print("euler angles from matrix: (roll, pitch, yaw)")
		print(euler_from_matrix(rotationMatrix))

		x = easting 
		y = northing
		height = alt
		utc = time

	# print('rotation matrix array size: ', rotationMatrix.size)
	# print('easting array size: ', x.size)
	# print('northing array size: ', y.size)
	# print('alt array size: ', height.size)
	# print('time array size: ', utc.size)
	
	# file output
	dataOutput = np.column_stack((output,easting,northing,alt,time))
	np.savetxt(args.output_file, dataOutput, fmt='%.5f' ,delimiter=' ') # 5 signifigant digits

	# victory
	print("file has been written to output directory")

if __name__=='__main__':
	main()