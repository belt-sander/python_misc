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
	data = np.genfromtxt(args.input, skip_header=1, delimiter=',')
	print('data imported')

	time = data[:,0]
	dt = data[:,1]

	std_error = np.std(dt)
	mean_error = np.mean(dt)

	print()
	print('std_error:', std_error)
	print('mean_error:', mean_error)

	plt.figure(1)
	plt.plot(time, dt, label='can dt (s)')
	plt.legend()

	plt.show()

if __name__ == '__main__':
	main()
