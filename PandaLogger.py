#!/usr/bin/env python

# can logger using comma panda

from __future__ import print_function
import argparse
import numpy as np
import binascii
import csv
import sys
from panda import Panda

def parse_args():
    arg_parser = argparse.ArgumentParser(description='log can data using comma.ai panda and export to txt')

    arg_parser.add_argument('-o',
                            '--outputFileArg', 
                            required=True, 
                            help='txt file output path')
    return arg_parser.parse_args()

def can_logger():
	args = parse_args()

	try:
		print("Trying to connect to Panda over USB...")
		p = Panda()

	except AssertionError:
		print("USB connection failed.")

	try:
   		outputfile = open(args.outputFileArg, 'wb')
   		csvwriter = csv.writer(outputfile)
   		csvwriter.writerow(['Bus', 'MessageID', 'Message', 'MessageLength'])
   		print("Writing csv file output.csv. Press Ctrl-C to exit...\n")

		bus0_msg_cnt = 0
		bus1_msg_cnt = 0
		bus2_msg_cnt = 0

		while True:
			can_recv = p.can_recv()

    		for address, _, dat, src  in can_recv:
        		csvwriter.writerow([str(src), str(hex(address)), "0x" + binascii.hexlify(dat), len(dat)])

		    	if src == 0:
		        	bus0_msg_cnt += 1
		    	elif src == 1:
		       		bus1_msg_cnt += 1
		    	elif src == 2:
		        	bus2_msg_cnt += 1

    			print("Message Counts... Bus 0: " + str(bus0_msg_cnt) + " Bus 1: " + str(bus1_msg_cnt) + " Bus 2: " + str(bus2_msg_cnt), end='\r')

	except KeyboardInterrupt:
		print("\nNow exiting. Final message Counts... Bus 0: " + str(bus0_msg_cnt) + " Bus 1: " + str(bus1_msg_cnt) + " Bus 2: " + str(bus2_msg_cnt))
		outputfile.close()

if __name__ == "__main__":
		can_logger()