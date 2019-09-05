#!/usr/bin/env python

### this tool allows playback of data in a terminal and physically output to can via comma panda
### currently limited to physical playback of one ID at a time

from __future__ import print_function
import argparse
import numpy as np
import time
from panda import Panda
import struct
import sys
import matplotlib.pyplot as plt

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
	arg_parser.add_argument('-pb',
							'--playbackBus',
							required=False,
							default=0,
							type=int,
							help='replay bus number if other than bus 0')
	arg_parser.add_argument('-g',
							'--grapher',
							required=False,
							default=False,
							help='show plots of stuff')
	return arg_parser.parse_args()

def main():
	args = parse_args()

	# connect to panda if relay is active
	if args.replay == 'True':
		try:
			print("Trying to connect to Panda over USB...")
			p = Panda()
		
			# clear can data buffers on panda
			p.can_clear(0xffff)
		
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

	# import text file
	canData = np.genfromtxt(args.input, skip_header=1, delimiter=',', dtype=str) 
	print("data has been imported")

	# num can packets
	numCanPackets = np.size(canData,0)
	print("num can samples: ", numCanPackets)

	# set up data aggregator value
	dataGraph = np.zeros((len(canData),1))
	idGraph = np.zeros((len(canData),1))
	message1 = np.zeros((len(canData),1)) # experiment value for plotting
	message2 = np.zeros((len(canData),1)) # experiment value for plotting
	message3 = np.zeros((len(canData),1)) # experiment value for plotting
	message4 = np.zeros((len(canData),1)) # experiment value for plotting
	message1byte = np.zeros((len(canData),1)) # experiment value for plotting
	message2byte = np.zeros((len(canData),1)) # experiment value for plotting
	message3byte = np.zeros((len(canData),1)) # experiment value for plotting
	message4byte = np.zeros((len(canData),1)) # experiment value for plotting
	message5byte = np.zeros((len(canData),1)) # experiment value for plotting
	message6byte = np.zeros((len(canData),1)) # experiment value for plotting
	message7byte = np.zeros((len(canData),1)) # experiment value for plotting
	message8byte = np.zeros((len(canData),1)) # experiment value for plotting				
	vehSpd = np.zeros((len(canData),1)) # reference for plots

	userMess1 = np.zeros((len(canData),1)) # reference for plots

	# show current mask called at terminal
	if args.mask is not None:
		print("args mask", (args.mask))
		maskint = int(args.mask, 0)

	# for loop for iterating through text file
	for i, row in enumerate (canData):
		busNum = row[0]
		messIden = row[1]
		data = row[2]
		length = row[3]

		messIdenInt = int(messIden,0)
		busNumInt = int(busNum,0)
		lengthInt = int(length,0)
		dataInt = int(data,0)

		### physical CAN replay via panda
		if args.replay == 'True':	
			if maskint == 0x800:
				dataStructOut = struct.pack('>Q',dataInt) # '>Q' argument == big endian long struct format
				p.can_send(messIdenInt,dataStructOut,args.playbackBus)
				# print([(busNumInt), (messIden), (data), (lengthInt)])
				time.sleep(args.sleep) # sleep

			elif messIdenInt == maskint:
				dataStructOut = struct.pack('>Q',dataInt) # '>Q' argument == big endian long struct format
				p.can_send(messIdenInt,dataStructOut,args.playbackBus)
				# print([(busNumInt), (messIden), (data), (lengthInt)])								
				time.sleep(args.sleep) # sleep
		
		### terminal playback / debugging ###
		elif args.replay == 'False':
			if maskint == 0x800:
				dataStructOut = struct.pack('>Q',dataInt) # '>Q' argument == big endian long struct format
				print([(busNumInt), (messIden), (data), (lengthInt)])			

				time.sleep(args.sleep) # sleep

			elif messIdenInt == maskint:
				dataStructOut = struct.pack('>Q',dataInt) # '>Q' argument == big endian 8 byte unsigned // '>I' argument == big endian 4 byte unsigned
				# print([(busNumInt), (messIden), (data), (lengthInt)])

				if messIden == '0x155': # wheel speed id
					b5 = '0x'+(data[:16])[12:] # motec byte offset 5, 16 bit
					wheelSpeed = int(b5,0)*(0.00999999978)
					print([(busNumInt), (messIden), (data), (lengthInt)])	
				# crc test // CORRECT FOR 0x488 ((byte0 + byte1 + byte2 + id + len)%256)
				elif messIden == '0x488': # ap lateral command id
					mysteryFactor = 0
					dataSum = mysteryFactor + messIdenInt + lengthInt + ord(dataStructOut[0]) + ord(dataStructOut[1]) + ord(dataStructOut[2]) + ord(dataStructOut[3]) + ord(dataStructOut[4]) + ord(dataStructOut[5]) + ord(dataStructOut[6])
					crc = dataSum%256
					print('crc is correct!', [(busNumInt), (messIden), (data), (lengthInt)])
	
					if crc != ord(dataStructOut[7]):
						print('wrong crc y0!!!!')
						print('sum: ', dataSum)
						print('this is calculated crc: ', crc, 'this is the last byte: ', ord(dataStructOut[7]))
						print('error: ', crc - ord(dataStructOut[7]))
						print([(busNumInt), (messIden), (data), (lengthInt)])				
				# crc test // CORRECT FOR 0x370 ((byte0 + byte1 + byte2 + byte3 + byte4 + byte5 + byte6 + id + len + -5)%256)
				elif messIden == '0x370': # ap lateral command id
					mysteryFactor = -5
					dataSum = mysteryFactor + messIdenInt + lengthInt + ord(dataStructOut[0]) + ord(dataStructOut[1]) + ord(dataStructOut[2]) + ord(dataStructOut[3]) + ord(dataStructOut[4]) + ord(dataStructOut[5]) + ord(dataStructOut[6])
					crc = dataSum%256
			
					if crc != ord(dataStructOut[7]):
						print('wrong crc y0!!!!')
						print('sum: ', dataSum)
						print('this is calculated crc: ', crc, 'this is the last byte: ', ord(dataStructOut[7]))
						print('error: ', crc - ord(dataStructOut[7]))
						print([(busNumInt), (messIden), (data), (lengthInt)])
				# crc test // CORRECT FOR 0x175 ((byte0 + byte1 + byte2 + byte3 + byte4 + byte5 + byte6 + id + len + -7)%256)
				elif messIden == '0x175': # ap lateral command id
					mysteryFactor = -7
					dataSum = mysteryFactor + messIdenInt + lengthInt + ord(dataStructOut[0]) + ord(dataStructOut[1]) + ord(dataStructOut[2]) + ord(dataStructOut[3]) + ord(dataStructOut[4]) + ord(dataStructOut[5]) + ord(dataStructOut[6])
					crc = dataSum%256
			
					if crc != ord(dataStructOut[7]):
						print('wrong crc y0!!!!')
						print('sum: ', dataSum)
						print('this is calculated crc: ', crc, 'this is the last byte: ', ord(dataStructOut[7]))
						print('error: ', crc - ord(dataStructOut[7]))
						print([(busNumInt), (messIden), (data), (lengthInt)])	

				time.sleep(args.sleep) # sleep

		### grapher for reversal / tests ###
		if args.grapher == 'True':
			### vehicle speed reference channel ###
			if messIden == '0x155':
				vehSpd[i:] = (int(('0x'+data[:16][12:]),0)&0xffff)*0.00999999978

			### data aggregators for plotting later
			if messIdenInt == maskint:
				dataGraph[i:] = dataInt
				idGraph[i:] = messIdenInt
				if lengthInt == 8:	
					message1[i:] = (int('0x'+data[:6][2:],0)&0xffff)
					message2[i:] = (int('0x'+data[:10][6:],0)&0xffff)
					message3[i:] = (int('0x'+data[:14][10:],0)&0xffff)
					message4[i:] = (int('0x'+data[:18][14:],0)&0xffff)				
					message1byte[i:] = (int('0x'+data[:4][2:],0)&0xff)						
					message2byte[i:] = (int('0x'+data[:6][4:],0)&0xff)
					message3byte[i:] = (int('0x'+data[:8][6:],0)&0xff)
					message4byte[i:] = (int('0x'+data[:10][8:],0)&0xff)
					message5byte[i:] = (int('0x'+data[:12][10:],0)&0xff)
					message6byte[i:] = (int('0x'+data[:14][12:],0)&0xff)										
					message7byte[i:] = (int('0x'+data[:16][14:],0)&0xff)										
					message8byte[i:] = (int('0x'+data[:18][16:],0)&0xff)	
					userMess1[i:] = (int('0x'+data[:8][2:],0)&0xffffff)														
				elif lengthInt == 6:
					message1[i:] = (int('0x'+data[:6][2:],0)&0xffff)
					message2[i:] = (int('0x'+data[:10][6:],0)&0xffff)
					message3[i:] = (int('0x'+data[:14][10:],0)&0xffff)					
					message1byte[i:] = (int('0x'+data[:4][2:],0)&0xff)						
					message2byte[i:] = (int('0x'+data[:6][4:],0)&0xff)
					message3byte[i:] = (int('0x'+data[:8][6:],0)&0xff)
					message4byte[i:] = (int('0x'+data[:10][8:],0)&0xff)
					message5byte[i:] = (int('0x'+data[:12][10:],0)&0xff)
					message6byte[i:] = (int('0x'+data[:14][12:],0)&0xff)										
				elif lengthInt == 4:
					message1[i:] = (int('0x'+data[:6][2:],0)&0xffff)
					message2[i:] = (int('0x'+data[:10][6:],0)&0xffff)
					message1byte[i:] = (int('0x'+data[:4][2:],0)&0xff)						
					message2byte[i:] = (int('0x'+data[:6][4:],0)&0xff)
					message3byte[i:] = (int('0x'+data[:8][6:],0)&0xff)
					message4byte[i:] = (int('0x'+data[:10][8:],0)&0xff)
				else:
					print('you need to finish writing yer c0de bruh...')
					sys.exit(0)

	### show plots ###	
	if args.grapher == 'True':		
		print('')		
		print('data after for loop: ', dataGraph, '\n')	
		print('data array size: ', np.size(dataGraph), '\n')		
		print('id after for loop: ', idGraph, '\n')
		print('id array size:', np.size(idGraph), '\n')
		print('message1 :', message1, '\n')
		print('message1 size: ', np.size(message1), '\n')
		print('')

		fig, (ax1, ax2, ax3, ax4, ax5) = plt.subplots(5,1, sharex=True)
		fig.suptitle('16 bit values (big endian)')
		ax1.plot(vehSpd, color='red', label='vehicle speed (0x155)')
		ax1.legend()
		ax2.plot(message1, label='2 byte / message1')
		ax2.legend()
		ax3.plot(message2, label='2 byte / message2')
		ax3.legend()
		ax4.plot(message3, label='2 byte / message3')
		ax4.legend()
		ax5.plot(message4, label='2 byte / message4')
		ax5.set_xlabel('samples')
		ax5.legend()

		fig2, (ax6, ax7, ax8, ax9, ax10) = plt.subplots(5,1, sharex=True)
		fig2.suptitle('8 bit values (graph 1)')
		ax6.plot(vehSpd, color='red', label='vehicle speed (0x155)')
		ax6.legend()
		ax7.plot(message1byte, label='byte 0')
		ax7.legend()
		ax8.plot(message2byte, label='byte 1')
		ax8.legend()
		ax9.plot(message3byte, label='byte 2')
		ax9.legend()
		ax10.plot(message4byte, label='byte 3')
		ax10.set_xlabel('samples')
		ax10.legend()	

		fig3, (ax11, ax12, ax13, ax14, ax15) = plt.subplots(5,1, sharex=True)
		fig3.suptitle('8 bit values (graph 2)')
		ax11.plot(vehSpd, color='red', label='vehicle speed (0x155)')
		ax11.legend()
		ax12.plot(message5byte, label='byte 4')
		ax12.legend()
		ax13.plot(message6byte, label='byte 5')
		ax13.legend()
		ax14.plot(message7byte, label='byte 6')
		ax14.legend()
		ax15.plot(message8byte, label='byte 7')
		ax15.set_xlabel('samples')
		ax15.legend()	

		fig4, (ax16, ax17) = plt.subplots(2,1, sharex=True)
		fig4.suptitle('user value guesses')
		ax16.plot(vehSpd, color='green', label='vehicle speed (0x155)')
		ax16.legend()
		ax17.plot(userMess1, label='user message guess')
		ax17.legend()
		ax17.set_xlabel('samples')

		plt.show()

	# re-enable safety mode when playback is complete
	if args.replay == 'True':
		p.set_safety_mode(p.SAFETY_NOOUTPUT)
		print('panda saftey mode re-enabled')
	else:
		print('playback finished...')

if __name__=='__main__':
	main()
