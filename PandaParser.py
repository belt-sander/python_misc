#!/usr/bin/python

import argparse
import numpy as np
import matplotlib.pyplot as plt
import binascii
import time

def parse_args():
	arg_parser = argparse.ArgumentParser(description='PandaLogger.py data parser')
	arg_parser.add_argument('-i',
							'--input', 
							required=True, 
							help='vectornav_output.txt file only!')
	return arg_parser.parse_args()

def main():
	args = parse_args()
	
	canData = np.genfromtxt(args.input, skip_header=1, delimiter=',')
	print("data has been imported")

	numCanPackets = np.size(canData,0)
	print("num can samples: ", numCanPackets)

	# busNum = canData[:,0]
	# messIden = canData[:,1]
	# data = canData[:,2]
	# length = canData[:,3]

	for i, row in enumerate(canData):
		busNum = row[0]
		messIden = row[1]
		data = row[2]
		length = row[3]
		print([(busNum), (messIden), (data), (length)])
		time.sleep(0.01)

if __name__=='__main__':
	main()
