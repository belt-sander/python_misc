#!/usr/bin/python

from __future__ import print_function
import argparse
import numpy as np
import matplotlib.pyplot as plt
import io

def parse_args():
	arg_parser = argparse.ArgumentParser(description='simple visualization tool data')
	arg_parser.add_argument('-i', '--input', required=True, help='input data')
	return arg_parser.parse_args()

def main():
	args = parse_args()
	dirty = io.BytesIO(open(args.input, 'rb').read().replace(b'*',b','))
	data = np.genfromtxt(dirty, delimiter=',', skip_header=10, skip_footer=10)

	# yaw = np.zeros((len(data),1))
	# pitch = np.zeros((len(data),1))
	# roll = np.zeros((len(data),1))
	# mag_x = np.zeros((len(data),1))
	# mag_y = np.zeros((len(data),1))
	# mag_z = np.zeros((len(data),1))
	# acc_x = np.zeros((len(data),1))
	# acc_y = np.zeros((len(data),1))
	# acc_z = np.zeros((len(data),1))
	# gyro_x = np.zeros((len(data),1))
	# gyro_y = np.zeros((len(data),1))
	# gyro_z = np.zeros((len(data),1))

	_yaw = data[:,1]
	_pitch = data[:,2]	
	_roll = data[:,3]
	_mag_x = data[:,4]
	_mag_y = data[:,5]
	_mag_z = data[:,6]
	_acc_x = data[:,7]
	_acc_y = data[:,8]
	_acc_z = data[:,9]
	_gyro_x = data[:,10]
	_gyro_y = data[:,11]
	_gyro_z = data[:,12]

	print('mean gyro_x: ', '%.6f' % np.mean(_gyro_x))
	print('mean gyro_y: ', '%.6f' % np.mean(_gyro_y))
	print('mean gyro_z: ', '%.6f' % np.mean(_gyro_z))

	gao_data, (gyro_x, gyro_y, gyro_z, ax, ay, az, y, p, r) = plt.subplots(9,1, sharex=True)
	gao_data.suptitle('gyro / accel / ypr')
	gyro_x.plot(_gyro_x, label='gyro x rad/sec')
	gyro_x.legend()
	gyro_y.plot(_gyro_y, label='gyro y rad/sec')
	gyro_y.legend()
	gyro_z.plot(_gyro_z, label='gyro z rad/sec')
	gyro_z.legend()
	ax.plot(_acc_x, label='accel x m/s/s')
	ax.legend()
	ay.plot(_acc_y, label='accel y m/s/s')
	ay.legend()
	az.plot(_acc_z, label='accel z m/s/s')
	az.legend()
	y.plot(_yaw, label='yaw degrees', color='Orange')
	y.legend()
	p.plot(_pitch, label='pitch degrees', color='Orange')
	p.legend()
	r.plot(_roll, label='roll degrees', color='Orange')
	r.legend()


	plt.show()

if __name__=='__main__':
	main()
