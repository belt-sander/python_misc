#!/usr/bin/python

import argparse
import numpy as np
import matplotlib.pyplot as plt
import binascii
import time
from panda import Panda

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
	arg_parser.add_argument('-o',
							'--textOutput',
							required=False,
							help='print data in terminal')

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
		
		if lengthInt == 8:
			byte0 = ((int(data,0) & 0xff00000000000000) >> 56)
			byte0hex = hex(byte0).rstrip("L")
			byte1 = ((int(data,0) & 0x00ff000000000000) >> 48)
			byte1hex = hex(byte1).rstrip("L")
			byte2 = ((int(data,0) & 0x0000ff0000000000) >> 40)
			byte2hex = hex(byte2).rstrip("L")
			byte3 = ((int(data,0) & 0x000000ff00000000) >> 32)
			byte3hex = hex(byte3).rstrip("L")
			byte4 = ((int(data,0) & 0x00000000ff000000) >> 24)
			byte4hex = hex(byte4).rstrip("L")
			byte5 = ((int(data,0) & 0x0000000000ff0000) >> 16)
			byte5hex = hex(byte5).rstrip("L")
			byte6 = ((int(data,0) & 0x000000000000ff00) >> 8)
			byte6hex = hex(byte6).rstrip("L")
			byte7 = (int(data,0) & 0x00000000000000ff)
			byte7hex = hex(byte7).rstrip("L")
			assy = '\\' + str(byte0hex)[1:] + '\\' + byte1hex[1:] + '\\' + byte2hex[1:] + '\\' + byte3hex[1:] + '\\' + byte4hex[1:] + '\\' + byte5hex[1:] + '\\' + byte6hex[1:] + '\\' + byte7hex[1:]
			print(assy)
			p.can_send(messIdenInt, assy, busNumInt)

		elif lengthInt == 7:
			byte0 = ((int(data,0) & 0xff00000000000000) >> 56)
			byte0hex = hex(byte0)
			byte1 = ((int(data,0) & 0x00ff000000000000) >> 48)
			byte1hex = hex(byte1)
			byte2 = ((int(data,0) & 0x0000ff0000000000) >> 40)
			byte2hex = hex(byte2)
			byte3 = ((int(data,0) & 0x000000ff00000000) >> 32)
			byte3hex = hex(byte3)
			byte4 = ((int(data,0) & 0x00000000ff000000) >> 24)
			byte4hex = hex(byte4)
			byte5 = ((int(data,0) & 0x0000000000ff0000) >> 16)
			byte5hex = hex(byte5)
			byte6 = ((int(data,0) & 0x000000000000ff00) >> 8)
			byte6hex = hex(byte6)

		elif lengthInt == 6:
			byte0 = ((int(data,0) & 0xff00000000000000) >> 56)
			byte0hex = hex(byte0)
			byte1 = ((int(data,0) & 0x00ff000000000000) >> 48)
			byte1hex = hex(byte1)
			byte2 = ((int(data,0) & 0x0000ff0000000000) >> 40)
			byte2hex = hex(byte2)
			byte3 = ((int(data,0) & 0x000000ff00000000) >> 32)
			byte3hex = hex(byte3)
			byte4 = ((int(data,0) & 0x00000000ff000000) >> 24)
			byte4hex = hex(byte4)
			byte5 = ((int(data,0) & 0x0000000000ff0000) >> 16)
			byte5hex = hex(byte5)

		elif lengthInt == 5:
			byte0 = ((int(data,0) & 0xff00000000000000) >> 56)
			byte0hex = hex(byte0)
			byte1 = ((int(data,0) & 0x00ff000000000000) >> 48)
			byte1hex = hex(byte1)
			byte2 = ((int(data,0) & 0x0000ff0000000000) >> 40)
			byte2hex = hex(byte2)
			byte3 = ((int(data,0) & 0x000000ff00000000) >> 32)
			byte3hex = hex(byte3)
			byte4 = ((int(data,0) & 0x00000000ff000000) >> 24)
			byte4hex = hex(byte4)

		elif lengthInt == 4:
			byte0 = ((int(data,0) & 0xff00000000000000) >> 56)
			byte0hex = hex(byte0)
			byte1 = ((int(data,0) & 0x00ff000000000000) >> 48)
			byte1hex = hex(byte1)
			byte2 = ((int(data,0) & 0x0000ff0000000000) >> 40)
			byte2hex = hex(byte2)
			byte3 = ((int(data,0) & 0x000000ff00000000) >> 32)
			byte3hex = hex(byte3)

		elif lengthInt == 3:
			byte0 = ((int(data,0) & 0xff00000000000000) >> 56)
			byte0hex = hex(byte0)
			byte1 = ((int(data,0) & 0x00ff000000000000) >> 48)
			byte1hex = hex(byte1)
			byte2 = ((int(data,0) & 0x0000ff0000000000) >> 40)
			byte2hex = hex(byte2)

		elif lengthInt == 2:
			byte0 = ((int(data,0) & 0xff00000000000000) >> 56)
			byte0hex = hex(byte0)
			byte1 = ((int(data,0) & 0x00ff000000000000) >> 48)
			byte1hex = hex(byte1)

		elif lengthInt == 1:
			byte0 = ((int(data,0) & 0xff00000000000000) >> 56)
			byte0hex = hex(byte0)

		# print([(busNumInt), (messIdenInt), (data), (lengthInt)])

		time.sleep(0.01)

if __name__=='__main__':
	main()
