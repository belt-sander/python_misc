#!/usr/bin/env python

from __future__ import print_function
import argparse
import numpy as np
import matplotlib.pyplot as plt

def parse_args():
	arg_parser = argparse.ArgumentParser(description='basic AF plotting script')
	arg_parser.add_argument('-i', '--input', required=True, help='input file from session.txt')
	return arg_parser.parse_args()

def main():
	args = parse_args()
	data = np.genfromtxt(args.input, skip_header=100, skip_footer=100)
	print('data imported')

	time = data[:,0]
	msl = data[:,3]
	hell = data[:,22]

	error = msl - hell
	std_error = np.std(error)
	mean_error = np.mean(error)

	print()
	print('std_error:', std_error)
	print('mean_error:', mean_error)

	plt.plot(time, error, label='msl - hEll (m)')
	plt.legend()
	plt.show()

if __name__ == '__main__':
	main()
