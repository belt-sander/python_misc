#!/usr/bin/python
import numpy as np
import argparse
from transformations import euler_from_matrix

def parse_args():
    arg_parser = argparse.ArgumentParser(description='global to body rotation test tool')

    arg_parser.add_argument('-i',
                            '--input_csv', 
                            required=True, 
                            help='input text file from MoTeCGPSTimeConvert.py')

    return arg_parser.parse_args()

def main():
	args = parse_args()
	inputData = np.genfromtxt(args.input_csv, skip_header=1, delimiter='')

	vnMatrix = np.zeros((len(inputData),9))

	# vnYaw = inputData[:,10]
	# vnPitch = inputData[:,11]
	# vnRoll = inputData[:,12]
	vnVelX = inputData[:,23]
	vnVelY = inputData[:,24]
	vnVelZ = inputData[:,25]

	print("")
	print("data hath been imp0rt3d")
	print("")

	# print("yaw: ", vnYaw)
	# print("pitch: ", vnPitch)
	# print("roll: ", vnRoll)
	print("velX: ", vnVelX)
	print("velY: ", vnVelY)
	print("velZ: ", vnVelZ)
	print("")

	# print("empty matrix: ", vnYawMatrix)
	# print("size: ", np.size(vnYawMatrix))

	for i, row in enumerate(inputData):
		yaw = row[10]/180*np.pi
		pitch = row[11]/180*np.pi
		roll = row[12]/180*np.pi
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
		vnMatrix[i,:] = rotationMatrix.reshape((1, 9))

		print("my matrix: ")
		print(rotationMatrix)

		# standard euler angle representation: roll, pitch, yaw
		print("vectornav euler (radians): (roll, pitch, yaw)")
		print(roll, pitch, yaw)

		print("euler angles from matrix (radians): (roll, pitch, yaw)")
		print(euler_from_matrix(rotationMatrix))


	# print("yaw matrix: ", vnYawMatrix)

if __name__=='__main__':
    main()