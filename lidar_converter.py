#!/usr/bin/env python3

import sys
import argparse
import numpy as np
import matplotlib.pyplot as plt
import time
import csv
from scipy.spatial.transform import Rotation as R

def parse_args():
	arg_parser = argparse.ArgumentParser(description='hdl-32e packet converter to xyzi')
	arg_parser.add_argument('-i', '--input',  required=True, help='raw text file in which you want to clean')
	arg_parser.add_argument('-o', '--output', required=False, help='place to save text file xyzi')
	return arg_parser.parse_args()

def main():
	args = parse_args()
	file = open(args.input, 'r')
	output_file = open(args.output, 'w')

	for row in csv.reader(file, quoting=csv.QUOTE_NONNUMERIC):
		time = (row[0] + (row[1]*1e-9))
		print('time stamp: ',time)
		print()
		print('raw: ', row)

if __name__=='__main__':
	main()