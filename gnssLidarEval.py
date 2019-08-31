#!/usr/bin/python

import argparse
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter

def parse_args():
	arg_parser = argparse.ArgumentParser(description='carrier to noise ratio plotter')
	arg_parser.add_argument('-i',
							'--input', 
							required=True, 
							help='minicom ascii output from <log range ontime 1> WITHOUT LIDAR POWERED')
	arg_parser.add_argument('-il',
							'--input_lidar', 
							required=True, 
							help='minicom ascii output from <log range ontime 1> WITH LIDAR POWERED')	
	return arg_parser.parse_args()

cNo = []
satNum = []

lidarCNo = []
lidarSatNum = []

def main():
	args = parse_args()

	# non lidar file parser
	with open(args.input) as f:
	    lis = [line.split() for line in f]        
	    for i, x in enumerate(lis):
	    	# print "line{0} = {1}".format(i, x), 'array size: ', np.size(x)

	    	if np.size(x) == 11:
	    		# print(x[8])
	    		cNo.append(x[8])
	    		satNum.append(x[1])
	
	# with lidar file parser
	with open(args.input_lidar) as f:
	    lis = [line.split() for line in f]        
	    for i, x in enumerate(lis):
	    	# print "line{0} = {1}".format(i, x), 'array size: ', np.size(x)

	    	if np.size(x) == 11:
	    		# print(x[8])
	    		lidarCNo.append(x[8])
	    		lidarSatNum.append(x[1])


	filtered_cNo = savgol_filter(cNo, 121, 2)
	filtered_lidarCNo = savgol_filter(lidarCNo, 121, 2)



	plt.figure(1)
	plt.hold(True)
	plt.plot(filtered_cNo, color='red', label='Filtered Carrier Noise Ratio (db-Hz)')
	plt.plot(filtered_lidarCNo, color='blue', label='Filtered Carrier Noise Ratio With LiDAR(db-Hz)')
	plt.title('cNo Compare ~1cm Apart')
	plt.ylim(30,50)
	plt.legend()

	plt.show()

if __name__=='__main__':
	main()
