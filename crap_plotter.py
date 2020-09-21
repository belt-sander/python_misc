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
	lf = data[:,1]
	rf = data[:,2]
	lr = data[:,3]
	rr = data[:,4]
	ubx = data[:,6]

	error = ((lr+rr)/2) - ubx
	std_error = np.std(error)
	mean_error = np.mean(error)

	print()
	print('std_error:', std_error)
	print('mean_error:', mean_error)

	plt.figure(1)
	plt.subplot(211)
	plt.plot(time, error, label='rear vel - ubx (m/s)')
	plt.legend()

	plt.subplot(212)
	plt.plot(time, lf, label='lf (m/s)')
	plt.plot(time, rf, label='rf (m/s)')
	plt.plot(time, lr, label='lr (m/s)')
	plt.plot(time, rr, label='rr (m/s)')
	plt.plot(time, ubx, label='ubx (m/s)')
	plt.legend()
	plt.show()

if __name__ == '__main__':
	main()
