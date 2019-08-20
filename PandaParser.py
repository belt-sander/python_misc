#!/usr/bin/env python

### this tool allows playback of data in a terminal and physically output to can via comma panda
### currently limited to physical playback of one ID at a time

import argparse
import numpy as np
import time
from panda import Panda
import struct
import sys

def parse_args():
	arg_parser = argparse.ArgumentParser(description='PandaLogger.py data parser')
	arg_parser.add_argument('-i',
							'--input', 
							required=True, 
							help='output file from PandaLogger.py')
	arg_parser.add_argument('-t',
							'--testMode',
							required=False,
							default=False,
							help='set to <True> if ack is required')	
	arg_parser.add_argument('-m',
							'--mask',
							required=False,
							default='0x800',
							help='play back data only on this identifier <decimal>')
	arg_parser.add_argument('-r',
							'--replay',
							required=True,
							default=False,
							help='play back physical data via CAN')
	arg_parser.add_argument('-s',
							'--sleep',
							required=True,
							type=float,
							help='sleep time (ms) between playing messages back')
	return arg_parser.parse_args()

def main():
	args = parse_args()

	if args.replay == 'True':
		try:
			print("Trying to connect to Panda over USB...")
			p = Panda()
		except AssertionError:
			print("USB connection failed.")
			sys.exit(0)

		# turn off CAN tx safety mode
		if args.testMode == 'True':
			p.set_safety_mode(p.SAFETY_ALLOUTPUT)
		elif args.testMode == 'False':
			p.set_safety_mode(p.SAFETY_NOOUTPUT)
		else:
			print("incorrect testMode arguement")
			sys.exit("exiting...")

	canData = np.genfromtxt(args.input, skip_header=1, delimiter=',', dtype=str	)
	print("data has been imported")

	numCanPackets = np.size(canData,0)
	print("num can samples: ", numCanPackets)

	if args.mask is not None:
		print("args mask", (args.mask))
		maskint = int(args.mask, 0)

	for i, row in enumerate (canData):
		busNum = row[0]
		messIden = row[1]
		data = row[2]
		length = row[3]

		messIdenInt = int(messIden,0)
		busNumInt = int(busNum,0)
		lengthInt = int(length,0)
		dataInt = int(data,0)

		if args.replay == 'True':	
			if maskint == 0x800:
				print([(busNumInt), (messIden), (data), (lengthInt)])
				time.sleep(args.sleep)
			elif messIdenInt == maskint:
				dataStructOut = struct.pack('>Q',dataInt) # '>Q' argument == big endian long struct format
				p.can_send(messIdenInt,dataStructOut,busNumInt)
				print([(busNumInt), (messIden), (data), (lengthInt)])
				time.sleep(args.sleep)
		
		elif args.replay == 'False':
			if maskint == 0x800:
				print([(busNumInt), (messIden), (data), (lengthInt)])
				time.sleep(args.sleep)
			elif messIdenInt == maskint:
				print([(busNumInt), (messIden), (data), (lengthInt)])
				time.sleep(args.sleep)				

	# re-enable safety mode
	p.set_safety_mode(p.SAFETY_NOOUTPUT)

if __name__=='__main__':
	main()
