#!/usr/bin/env python3

import sys
import argparse
import numpy as np
import time

def parse_args():
	arg_parser = argparse.ArgumentParser(description='simple file cleaner')
	arg_parser.add_argument('-i', '--input',  required=True, help='raw text file in which you want to clean')
	arg_parser.add_argument('-o', '--output', required=True, help='place to save text files')
	arg_parser.add_argument('-d', '--debug', required=False, default='False', help='print output of original text file, line by line')
	return arg_parser.parse_args()

def main():
	args = parse_args()
	file = open(args.input, 'r')
	cleaned_file = open(args.output, 'w')
	for line in file:
		output_a = line.replace('"""','')
		output_b = output_a.replace(',,',',')
		output_c = output_b.replace('"','')
		output_d = output_c.replace('[','')
		output_e = output_d.replace(']','')
		output_f = output_e.replace('(','')
		output_g = output_f.replace(')','')
		output_h = output_g.replace('#', ' 0x')

		if args.debug == 'True':
			print(output_h)
			time.sleep(1.0)
		

		# write to new text file
		cleaned_file.write(output_h)
	print('done.')

if __name__=='__main__':
	main()