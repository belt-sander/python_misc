#!/usr/bin/python
import numpy as np
import matplotlib.pyplot as plt
import argparse

def parse_args():
	arg_parser = argparse.ArgumentParser(description='add/subtract offsets to relative UTM positions in normal trajectory file format. ptc inner: x=-541254, y=-5367997 / ptc outer_a: x=-541218, y=-5367960 / ptc outer_b: x=-541219, y=-5367961') 
	arg_parser.add_argument('--input_file',
							required=True,
							help='input file in mapper trajectory format')
	arg_parser.add_argument('--output_file',
							required=True,
							help='output file in mapper trajectory format with offsets applied')
	arg_parser.add_argument('--x_offset',
							required=True,
							type=float,
							help='float to add or subtract to x position')
	arg_parser.add_argument('--y_offset',
							required=True,
							type=float,
							help='float to add or subtract to y position')
	return arg_parser.parse_args()

def main():
	# data input
	args = parse_args()
	inputData = np.genfromtxt(args.input_file)

	# label data from input file
	rm_1 = np.float32(inputData[:,0])
	rm_2 = np.float32(inputData[:,1])
	rm_3 = np.float32(inputData[:,2])
	rm_4 = np.float32(inputData[:,3])
	rm_5 = np.float32(inputData[:,4])
	rm_6 = np.float32(inputData[:,5])
	rm_7 = np.float32(inputData[:,6])
	rm_8 = np.float32(inputData[:,7])
	rm_9 = np.float32(inputData[:,8])
	latitude = np.float32(inputData[:,9])
	longitude = np.float32(inputData[:,10])
	alt = np.float32(inputData[:,11])
	time = np.float32(inputData[:,12])

	#rw relative offsets
	rw_x = args.x_offset
	rw_y = args.y_offset

	size = inputData.shape
	print('array size:')
	print(size)

	# add offset to x/y utm
	for input in inputData:
		newlat = latitude+rw_x
		newlong = longitude+rw_y

	data=np.column_stack((rm_1,rm_2,rm_3,rm_4,rm_5,rm_6,rm_7,rm_8,rm_9,newlat,newlong,alt,time))
	print('data has had offset applied to it')
	
	np.savetxt(args.output_file, data, fmt='%.5f')
	print('new file has been saved in local directory')

if __name__=='__main__':
	main()