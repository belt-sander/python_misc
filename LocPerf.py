#!/usr/bin/python

import argparse
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter

def parse_args():
	arg_parser = argparse.ArgumentParser(description='calculate localization pose update rate from a localization_trajectory.txt file')
	arg_parser.add_argument('--input_file_x', 
							required=True, 
							help='trajectory input file xavier')
	arg_parser.add_argument('--input_file_s76',
							required=True,
							help='trajectory file input system76')
	return arg_parser.parse_args()

def main():
	args = parse_args()
	inputData = np.genfromtxt(args.input_file_x, delimiter='')
	inputDataS76 = np.genfromtxt(args.input_file_s76, delimiter='')
	
	print("")
	print("data has been imported")
	print("")

	### Xavier data input
	poseTime = inputData[:,12]
	resizedPoseTime = poseTime[1:]
	# print(poseTime)
	poseTimeDiff = np.diff(poseTime)
	# print(poseTimeDiff)
	poseRate = 1.0/poseTimeDiff
	# print(poseRate)
	filteredPoseRate = savgol_filter(poseRate, 51, 3)
	# print(filteredPoseRate)

	### System76 data input
	poseTimeS76 = inputDataS76[:,12]
	resizedPoseTimeS76 = poseTimeS76[1:]
	# print(poseTime)
	poseTimeDiffS76 = np.diff(poseTimeS76)
	# print(poseTimeDiff)
	poseRateS76 = 1.0/poseTimeDiffS76
	# print(poseRate)
	filteredPoseRateS76 = savgol_filter(poseRateS76, 51, 3)
	# print(filteredPoseRate)	

	poseRateMedian = np.median(poseRate)
	print('median localization pose frequency: ')
	print(poseRateMedian)
	print("")
	poseRateMedianS76 = np.median(poseRateS76)
	print('median s76 localization pose frequency: ')
	print(poseRateMedianS76)
	print("")

	plt.subplot(2,1,1)
	plt.title('system76 localization frequency')
	plt.plot(resizedPoseTimeS76, poseRateS76, label='raw pose frequency' ,color='orange')
	plt.plot(resizedPoseTimeS76, filteredPoseRateS76, label='filtered pose frequency' ,color='green')
	plt.ylim(0,10)
	plt.ylabel('localization frequency (hz)')
	plt.legend()

	plt.subplot(2,1,2)
	plt.title('xavier localization frequency')
	plt.plot(resizedPoseTime, poseRate, label='raw pose frequency' ,color='blue')
	plt.plot(resizedPoseTime, filteredPoseRate, label='filtered pose frequency' ,color='red')
	plt.ylim(0,10)
	plt.ylabel('localization frequency (hz)')
	plt.xlabel('utc time (s)')
	plt.legend()

	plt.show()

if __name__=='__main__':
	main()
