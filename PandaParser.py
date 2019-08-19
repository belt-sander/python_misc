#!/usr/bin/python

### this tool allows playback of data in a terminal and physically output to can via comma panda
### currently limited to physical playback of one ID at a time

import argparse
import numpy as np
import matplotlib.pyplot as plt
import binascii
import time
from panda import Panda
import struct

def parse_args():
	arg_parser = argparse.ArgumentParser(description='PandaLogger.py data parser')
	arg_parser.add_argument('-i',
							'--input', 
							required=True, 
							help='output file from PandaLogger.py')
	arg_parser.add_argument('-t',
							'--testMode',
							required=True,
							help='set to <True> if ack is required')	
	arg_parser.add_argument('-m',
							'--mask',
							required=False,
							help='play back data only on this identifier <decimal>')
	arg_parser.add_argument('-s',
							'--sleep',
							required=True,
							type=float,
							help='sleep time (ms) between playing messages back')

	return arg_parser.parse_args()

def main():
	args = parse_args()

	try:
		print("Trying to connect to Panda over USB...")
		p = Panda()
	except AssertionError:
		print("USB connection failed.")
		try:
			p = Panda("WIFI")
		except:
			print("WiFi connection timed out. Please make sure your Panda is connected and try again.")
			sys.exit(0)

	# turn off safety mode
	if args.testMode == "True":
		p.set_safety_mode(p.SAFETY_ALLOUTPUT)
	elif args.testMode == "False":
		p.set_safety_mode(p.SAFETY_NOOUTPUT)
	else:
		print("incorrect testMode arguement")
		sys.exit("exiting...")

	# set bus speeds. nominally 500kbps
	# p.set_can_speed_kbps(0,1000)	
	# p.set_can_speed_kbps(1,1000)
	# p.set_can_speed_kbps(2,1000)	

	canData = np.genfromtxt(args.input, skip_header=1, delimiter=',', dtype=str	)
	print("data has been imported")

	numCanPackets = np.size(canData,0)
	print("num can samples: ", numCanPackets)

	print("args mask", (args.mask))
	maskint = int(args.mask, 0)

	# busNum = canData[:,0]
	# messIden = canData[:,1]
	# data = canData[:,2]
	# length = canData[:,3]

	for i, row in enumerate (canData):
		busNum = row[0]
		messIden = row[1]
		data = row[2]
		length = row[3]

		messIdenInt = int(messIden,0)
		busNumInt = int(busNum,0)
		lengthInt = int(length,0)
		dataInt = int(data,0)

		if messIdenInt == maskint:
			dataStructOut = struct.pack('>Q',dataInt)
			p.can_send(messIdenInt,dataStructOut,busNumInt)
			print([(busNumInt), (messIdenInt), (data), (lengthInt)])
			time.sleep(args.sleep)
		elif args.mask is None:
			print([(busNumInt), (messIdenInt), (data), (lengthInt)])
			time.sleep(args.sleep)



if __name__=='__main__':
	main()
