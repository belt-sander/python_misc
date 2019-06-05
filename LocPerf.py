#!/usr/bin/python

import argparse
import numpy as np
import pandas as pd
import datetime
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def parse_args():
	arg_parser = argparse.ArgumentParser(description='add or subtract Z height from trajectory file')
	arg_parser.add_argument('--input_file', 
							required=True, 
							help='trajectory input file')
	return arg_parser.parse_args()

def main():
	args = parse_args()
	
	inputData = np.genfromtxt(args.input_file, skip_header=30, skip_footer=30, delimiter='')
	df = pd.read_csv(args.input_file)
	print("")
	print("data has been imported")
	print("")
if __name__=='__main__':
	main()