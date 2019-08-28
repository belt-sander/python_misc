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
				dataStructOut = struct.pack('>Q',dataInt) # '>Q' argument == big endian long struct format
				# print([(busNumInt), (messIden), (data), (lengthInt)])
				p.can_send(messIdenInt,dataStructOut,busNumInt)
				time.sleep(args.sleep)
			elif messIdenInt == maskint:
				dataStructOut = struct.pack('>Q',dataInt) # '>Q' argument == big endian long struct format
				p.can_send(messIdenInt,dataStructOut,busNumInt)
				# print([(busNumInt), (messIden), (data), (lengthInt)])								
				time.sleep(args.sleep)
		
		elif args.replay == 'False':
			if maskint == 0x800:
				dataStructOut = struct.pack('>Q',dataInt) # '>Q' argument == big endian long struct format
				print([(busNumInt), (messIden), (data), (lengthInt)])
				time.sleep(args.sleep)

			elif messIdenInt == maskint:
				dataStructOut = struct.pack('>Q',dataInt) # '>Q' argument == big endian 8 byte unsigned // '>I' argument == big endian 4 byte unsigned
				# print([(busNumInt), (messIden), (data), (lengthInt)])
				
				###
				# data manipulation (2 byte chunks)
				###
				data2byteOut = struct.unpack(b'4H',dataStructOut)
				print('wh spd? : ', data2byteOut[2])
				
				###
				# crc test // CORRECT FOR 0x488 ((byte0 + byte1 + byte2 + id + len)%256)
				# crc test // CORRECT FOR 0x370 ((byte0 + byte1 + byte2 + byte3 + byte4 + byte5 + byte6 + id + len + -5)%256)
				# crc test // CORRECT FOR 0x175 ((byte0 + byte1 + byte2 + byte3 + byte4 + byte5 + byte6 + id + len + -7)%256)
				###

				# mysteryFactor = -5
				# dataSum = mysteryFactor + messIdenInt + lengthInt + ord(dataStructOut[0]) + ord(dataStructOut[1]) + ord(dataStructOut[2]) + ord(dataStructOut[3]) + ord(dataStructOut[4]) + ord(dataStructOut[5]) + ord(dataStructOut[6])
				# crc = dataSum%256
				# if crc != ord(dataStructOut[7]):
				# 	print('wrong crc y0!!!!')
				# 	print('sum: ', dataSum)
				# 	print('this is calculated crc: ', crc, 'this is the last byte: ', ord(dataStructOut[7]))
				# 	print('error: ', crc - ord(dataStructOut[7]))
				# 	print([(busNumInt), (messIden), (data), (lengthInt)])

				time.sleep(args.sleep)				

	# re-enable safety mode
	if args.replay == 'True':
		p.set_safety_mode(p.SAFETY_NOOUTPUT)
		print('panda saftey mode re-enabled')
	else:
		print('playback finished...')

if __name__=='__main__':
	main()
