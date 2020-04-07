#!/usr/bin/env python3

import sys
import argparse
import numpy as np
import matplotlib.pyplot as plt
import time
from scipy.spatial.transform import Rotation as R

def parse_args():
	arg_parser = argparse.ArgumentParser(description='simple visualization tool data')
	arg_parser.add_argument('-i', '--input', required=True, help='data in which you want plotted')
	arg_parser.add_argument('-t', '--type', required=True, help='which data file type (imu, gps, ...)')
	return arg_parser.parse_args()


def main():
	args = parse_args()
	data = np.genfromtxt(args.input, delimiter=',', skip_header=1)

	if args.type == 'gps':
		_lat = data[:,7]
		_lon = data[:,8]
		_alt = data[:,9]

		_secs = data[:,2]
		_nsec = data[:,3]
		time = np.zeros((len(data),1))
		time = _secs + (_nsec * 1e-9)

		plt.subplot(211)
		plt.plot(_lat, _lon, label='position lla')
		plt.legend()

		plt.subplot(212)
		plt.plot(time, _alt, label='altitude')
		plt.legend()
		plt.show()

	elif args.type == 'imu':
		_quatx = data[:,5]
		_quaty = data[:,6]
		_quatz = data[:,7]
		_quatw = data[:,8]
		_gX = data[:,18]
		_gY = data[:,19]
		_gZ = data[:,20]
		_aX = data[:,30]
		_aY = data[:,31]
		_aZ = data[:,32]

		_secs = data[:,2]
		_nsec = data[:,3]
		time = np.zeros((len(data),1))
		time = _secs + (_nsec * 1e-9)

		plt.subplot(311)
		plt.plot(time, _quatx, label='quat x')
		plt.plot(time, _quaty, label='quat y')
		plt.plot(time, _quatz, label='quat z')
		plt.plot(time, _quatw, label='quat w')
		plt.legend()

		plt.subplot(312)
		plt.plot(time, _gX, label='gyro x')
		plt.plot(time, _gY, label='gyro y')
		plt.plot(time, _gZ, label='gyro z')
		plt.legend()

		plt.subplot(313)
		plt.plot(time, _aX, label='accel x')
		plt.plot(time, _aY, label='accel y')
		plt.plot(time, _aZ, label='accel z')
		plt.legend()
		plt.show()

	elif args.type == 'velocity':
		_vX = data[:,5]
		_vY = data[:,6]
		_vZ = data[:,7]

		_secs = data[:,2]
		_nsec = data[:,3]
		time = np.zeros((len(data),1))
		time = _secs + (_nsec * 1e-9)

		plt.plot(time, _vX, label='vel x')
		plt.plot(time, _vY, label='vel y')
		plt.plot(time, _vZ, label='vel z')						
		plt.legend()
		plt.show()

	else:
		print('bruh you spelled s0mtehin wr0ng.')
		sys.exit(0)

if __name__=='__main__':
	main()