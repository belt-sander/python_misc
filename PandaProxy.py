#!/usr/bin/env python

### proxy forwarder test

from panda import Panda
import argparse
import sys, traceback
import time
import binascii
import struct

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
		
		p.can_clear(0xffff)
		print('can ringbuffer cleared')
		
	except AssertionError:
		print("USB connection failed.")
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
	p.can_clear(0xffff)
	print('can ringbuffer cleared')

	### can forwarder settings ###
	# p.set_can_forwarding(0,1)
	p.set_can_forwarding(1,0)
	print('forwarder enabled can_1 -> can_0')

	### data format below ###
	# ('input data: ', [(1638, 59728, bytearray(b'\xff\xff\x00\x00\x00\x00\x00\x00'), 0)])
	# ('converted output: ', ['0', '0x666', '0xffff000000000000', 8])
	# ('addr: ', 1638)
	# ('dat: ', '\xff\xff\x00\x00\x00\x00\x00\x00')

	count = 0
	countData2 = 0
	newSend = False
	newSendData2 = False

	while True:
		inputData = p.can_recv()
		for address, _, dat, src  in inputData:
			if src == 0 and address == 0x666:
				newSend = True				
				if newSend == True:
					print'input data: ', inputData
					print'converted output: ', [str(src), str(hex(address)), "0x" + binascii.hexlify(dat), len(dat)]		
					print'addr: ', address
					print'send state: ', newSend					
					count = count + 1
					print'counter: ', count

					### simple test moving byte 0,1 to positon 6,7 ###
					db = '0x' + binascii.hexlify(dat)
					dbm = '0x'+'00'+'00'+'00'+'00'+'00'+'00'+db[:6][2:]
					dbmi = int(dbm,0)
					dbmip = struct.pack('>Q', dbmi)
					p.can_send(address, dbmip,1)

			if src == 0 and address == 0x034:
				newSendData2 = True
				if newSendData2 == True:
					print'input data: ', inputData
					print'converted output: ', [str(src), str(hex(address)), "0x" + binascii.hexlify(dat), len(dat)]		
					print'addr: ', address
					print'send state: ', newSendData2					
					countData2 = countData2 + 1
					print'counter: ', count

					### just a basic forwarder ###
					p.can_send(address, str(dat),1)					

if __name__ == "__main__":
		main()