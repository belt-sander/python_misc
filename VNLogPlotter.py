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

	_num = np.zeros((len(data),1),dtype=int)

	for i, row in enumerate(data):
		_num[i,:] = i

	_num_resize = np.resize(_num, (1, np.size(data)))

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

	gyro_x_bias = np.mean(_gyro_x)
	gyro_y_bias = np.mean(_gyro_y)
	gyro_z_bias = np.mean(_gyro_z)

	ax_bias = np.mean(_acc_x)
	ay_bias = np.mean(_acc_y)
	az_bias = np.mean(_acc_z)

	int_roll = np.cumsum(_gyro_x) * 50.0
	int_roll_bias_corr = (np.cumsum(_gyro_x - gyro_x_bias)) + _roll[0]

	int_pitch = np.cumsum(_gyro_y) * 50.0
	int_pitch_bias_corr = (np.cumsum(_gyro_y - gyro_y_bias)) + _pitch[0]

	int_yaw = np.cumsum(_gyro_z) * 50.0
	int_yaw_bias_corr = (np.cumsum(_gyro_z - gyro_z_bias)) + _yaw[0]


	print('mean gyro_x: ', '%.6f' % np.mean(_gyro_x))
	print('mean gyro_y: ', '%.6f' % np.mean(_gyro_y))
	print('mean gyro_z: ', '%.6f' % np.mean(_gyro_z))
	print('mean acc_x: ', '%.6f' % ax_bias)
	print('mean acc_y: ', '%.6f' % ay_bias)
	print('mean acc_z: ', '%.6f' % az_bias)

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
	y.plot(_yaw, label='yaw degrees', color='orange')
	y.plot(int_yaw_bias_corr, label='integrated yaw deg minus bias')
	y.legend()
	p.plot(_pitch, label='pitch degrees', color='orange')
	p.plot(int_pitch_bias_corr, label='integrated pitch deg minus bias')
	p.legend()
	r.plot(_roll, label='roll degrees', color='orange')
	r.plot(int_roll_bias_corr, label='integrated roll deg minus bias')
	r.legend()

	fig, (gx, gy, gz) = plt.subplots(3,1, sharex=True, sharey=True)
	fig.suptitle('gyro biases')
	gx.hist(_gyro_x,100, label='gyro x', density=True, facecolor='g', alpha=0.75)
	gx.legend()
	gy.hist(_gyro_y,100, label='gyro y', density=True, facecolor='r', alpha=0.75)
	gy.legend()
	gz.hist(_gyro_z,100, label='gyro z', density=True, facecolor='b', alpha=0.75)
	gz.legend()

	fig2, (ax, ay, az) = plt.subplots(3,1, sharex=False, sharey=True)
	fig2.suptitle('accel biases')
	ax.hist(_acc_x,100, label='accel x', density=True, facecolor='g', alpha=0.75)
	ax.legend()
	ay.hist(_acc_y,100, label='accel y', density=True, facecolor='r', alpha=0.75)
	ay.legend()
	az.hist(_acc_z,100, label='accel z', density=True, facecolor='b', alpha=0.75)
	az.legend()

	plt.show()



if __name__=='__main__':
	main()
