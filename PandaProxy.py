#!/usr/bin/env python

### proxy forwarder test

from panda import Panda
import argparse
import sys
import time
import binascii

def parse_args():
	arg_parser = argparse.ArgumentParser(description='recieved can data forwarder from bus 0 to bus 1')
	arg_parser.add_argument('-t',
							'--testMode',
							required=True,
							help='set to <True> if ack is required')
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

	print('ready for data...')
	newMess = False;

	while True:
		inputData = p.can_recv()
		for address, _, dat, src  in inputData:
			if address is not None:

				print('converted output: ', [str(src), str(hex(address)), "0x" + binascii.hexlify(dat), len(dat)])		
				print('addr: ', address)
				print('dat: ', str(dat))
				newMess = True
				
				if newMess == True:
					p.can_send(0x555,'sander',1)	
					newMess == False	

if __name__ == "__main__":
		main()