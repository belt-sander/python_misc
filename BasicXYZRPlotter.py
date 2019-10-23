#!/usr/bin/python

import argparse
import numpy as np
import datetime
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def parse_args():
	arg_parser = argparse.ArgumentParser(description='simple visualization tool for xyzr data')
	arg_parser.add_argument('-r', '--rows', required=True, type=int, help='number of rows to process')
	arg_parser.add_argument('-s1', '--scan_1', required=True, help='xyzr scan 1')
	arg_parser.add_argument('-s2', '--scan_2', required=False, help='xyzr scan 2')
	arg_parser.add_argument('-s3', '--scan_3', required=False, help='xyzr scan 3')
	arg_parser.add_argument('-s4', '--scan_4', required=False, help='xyzr scan 4')
	return arg_parser.parse_args()

def main():
	args = parse_args()
	
	scan1 = np.genfromtxt(args.scan_1)
	# scan2 = np.genfromtxt(args.scan_2)
	# scan3 = np.genfromtxt(args.scan_3)
	# scan4 = np.genfromtxt(args.scan_4)
	print("")
	print("data has been imported")
	print("")

	row_num = np.zeros((len(scan1),1))
	x1_sub = np.zeros((len(scan1),1))
	y1_sub = np.zeros((len(scan1),1))
	z1_sub = np.zeros((len(scan1),1))

	for i, row in enumerate(scan1):
		x1 = row[0]
		y1 = row[1]
		z1 = row[2]

		x1_sub[i,:] = x1
		y1_sub[i,:] = y1
		z1_sub[i,:] = z1			

		if i > args.rows:
			break

	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	ax.scatter(x1_sub, y1_sub, z1_sub)

	plt.show()

if __name__=='__main__':
	main()
