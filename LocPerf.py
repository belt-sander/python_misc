#!/usr/bin/python

import argparse
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter

def parse_args():
	arg_parser = argparse.ArgumentParser(description='calculate localization pose update rate from a localization_trajectory.txt file')
	arg_parser.add_argument('--input_file', 
							required=True, 
							help='trajectory input file')
	return arg_parser.parse_args()

def main():
	args = parse_args()
	inputData = np.genfromtxt(args.input_file, delimiter='')
	
	print("")
	print("data has been imported")
	print("")

	poseTime = inputData[:,12]
	resizedPoseTime = poseTime[1:]
	# print(poseTime)
	poseTimeDiff = np.diff(poseTime)
	# print(poseTimeDiff)
	poseRate = 1.0/poseTimeDiff
	# print(poseRate)
	filteredPoseRate = savgol_filter(poseRate, 51, 3)
	# print(filteredPoseRate)

	poseRateMean = np.mean(poseRate)
	print('mean localization pose frequency: ') 
	print(poseRateMean) 
	print("")

	poseRateMedian = np.median(poseRate)
	print('median localization pose frequency: ')
	print(poseRateMedian)
	print("")

	plt.subplot(2,1,1)
	plt.title('xavier localization frequency')
	plt.plot(resizedPoseTime, poseTimeDiff, '-o', color='gold')
	plt.ylim(0,1)
	plt.ylabel('time difference between poses (s)')
	# plt.xlim(0,600)

	plt.subplot(2,1,2)
	plt.plot(resizedPoseTime, poseRate, label='raw pose frequency' ,color='blue')
	plt.plot(resizedPoseTime, filteredPoseRate, label='filtered pose frequency' ,color='red')
	plt.ylim(0,10)
	plt.ylabel('localization frequency (hz)')
	plt.xlabel('utc time (s)')
	plt.legend()

	# plt.xlim(0,600)

	plt.show()

if __name__=='__main__':
	main()
