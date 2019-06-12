#!/usr/bin/python

import serial
import time
import csv
import argparse
import os
import numpy as np

def parse_args():
    arg_parser = argparse.ArgumentParser(description='configure vectornav vn200 to log data')

    arg_parser.add_argument('-i',
                            '--input_port', 
                            required=True, 
                            help='serial port that VectorNav is connected to (e.g. "/dev/ttyUSB0")')
    arg_parser.add_argument('-b',
                            '--baudrate',
                            required=False,
                            default='115200',
                            help='current VectorNav baud rate (default: 115200)')
    arg_parser.add_argument('-o',
                            '--output',
                            required=False,
                            help='output log file location')
    arg_parser.add_argument('-f',
                            '--data_freq',
                            required=False,
                            default='40',
                            #type=int,
                            help='requested VectorNav data frequency int (default: 40) (40hz)')
    arg_parser.add_argument('-n',
                            '--nothing',
                            action='store_true',
                            required=False,
                            default=False,
                            help='no printed terminal output. only use if l33t h4ck3r.')
    arg_parser.add_argument('-R',
    						'--FACTORY_RESET',
    						action='store_true',
    						required=False,
    						default=False,
    						help='sensor factory reset. only use this if you have no other options.')

    return arg_parser.parse_args()

def sensor_reset():
	args = parse_args()

	### sensor reset command
	sensorReset = ('$VNRFS*5F').encode()

	return sensorReset

def sensor_baud():
    args = parse_args()

    ### set sensor baud rate
    sensorBaudHeader = ('$VNWRG,05,').encode()
    baudRate = (args.baudrate)
    CRC = (',1*XX').encode()
    newLineCarRet = ('\r \n').encode()

    return sensorBaudHeader, baudRate, CRC, newLineCarRet

def sensor_data_binary():
    args = parse_args()

    ### set binary data type (serial port 1, 80/800 = 10hz, group 10 (attitude), group 10 number 0002 (ypr), *CRC)
    dataConfigHeader = ('$VNWRG,75,').encode()
    dataAsyncMode = ('1,').encode() # data sent out on serial port 1
    dataRateDivisor = ('80,').encode() # number divided by 800 == frequency (10hz)
    dataOutputGroup = ('10,').encode() # selected binary output group number
    dataOutputField = ('0002').encode() # selected binary data in dataOutputGroup
    CRC = (',1*XX').encode()
    
    newLineCarRet = ('\r \n').encode()

    ### test message
    dataAsyncDisable = ('$VNWRG,06,0,1*XX').encode()
    dataTest = ('$VNWRG,75,1,800,01,0001*XX').encode()

    return dataConfigHeader, dataAsyncMode, dataRateDivisor, dataOutputGroup, dataOutputField, CRC, newLineCarRet, dataAsyncDisable ,dataTest

def sensor_freq():
    args = parse_args()
    
    ### set data update rate    
    freqConfigHeader = ('$VNWRG,07,').encode()
    freqValue = (args.data_freq)
    CRC = (',1*XX').encode()
    newLineCarRet = ('\r \n').encode()        

    return freqConfigHeader, freqValue, CRC, newLineCarRet
    
def main():
    args = parse_args()
    ser = serial.Serial()               
    
    ser.port = args.input_port              #serial port path
    ser.baudrate = 115200                   #115200 == default VN baud rate
    ser.bytesize = serial.EIGHTBITS         #number of bits per bytes
    ser.parity = serial.PARITY_NONE         #set parity check: no parity
    ser.stopbits = serial.STOPBITS_TWO      #number of stop bits
    ser.timeout = 5                         #non-block read
    ser.xonxoff = False                     #disable software flow control
    ser.rtscts = False                      #disable hardware (RTS/CTS) flow control
    ser.dsrdtr = False                      #disable hardware (DSR/DTR) flow control    

    try: 
        ser.open()
        ser.flush()
        
    except Exception, e:
        print "error open serial port: " + str(e)
        exit()

    freqConfigHeader, freqValue, CRC, newLineCarRet = sensor_freq()
    dataConfigHeader, dataAsyncMode, dataRateDivisor, dataOutputGroup, dataOutputField, CRC, newLineCarRet, dataTest, dataAsyncDisable = sensor_data_binary()
    sensorBaudHeader, baudRate, CRC, newLineCarRet = sensor_baud()
    sensorReset = sensor_reset()
    
    if args.FACTORY_RESET == True:
    	ser.write(sensorReset)
    	print('')
    	print('sensor has been reset to factory settings!')

    ser.write(sensorBaudHeader)
    ser.write(baudRate)
    ser.write(CRC)
    ser.write(newLineCarRet)

    # ser.write(dataConfigHeader)
    # ser.write(dataAsyncMode)
    # ser.write(dataRateDivisor)
    # ser.write(dataOutputGroup)
    # ser.write(dataOutputField)
    # ser.write(CRC)
    # ser.write(newLineCarRet)

    # ser.write(freqConfigHeader)
    # ser.write(freqValue)
    # ser.write(CRC)
    # ser.write(newLineCarRet)

    ser.write(dataAsyncDisable)
    ser.write(newLineCarRet)

    ser.write(dataTest)
    ser.write(newLineCarRet)

    if int(args.baudrate) != 115200:
        ser.flush()
        ser.close()

        ser.port = args.input_port              #serial port path
        ser.baudrate = int(args.baudrate)       #new baud rate
        ser.bytesize = serial.EIGHTBITS         #number of bits per bytes
        ser.parity = serial.PARITY_NONE         #set parity check: no parity
        ser.stopbits = serial.STOPBITS_TWO      #number of stop bits
        ser.timeout = 5                         #non-block read
        ser.xonxoff = False                     #disable software flow control
        ser.rtscts = False                      #disable hardware (RTS/CTS) flow control
        ser.dsrdtr = False                      #disable hardware (DSR/DTR) flow control    
        print('')
        print('changed local serial port baud from 115200 to ' + args.baudrate)

        try: 
            ser.open()
            ser.flush()
        
        except Exception, e:
            print 'error open serial port: ' + str(e)
            exit()

    print('')
    print('configure complete, w00t.')
    print('')

    ### data type tests:
    ### custom_dtype = np.dtype([('header', np.uint8), ('group', np.uint8), ('fields', np.uint16), ('yaw', np.float), ('pitch', np.float), ('roll', np.float), ('crc', np.uint16)]) 

    if ser.isOpen():
        # file = open(os.path.join(args.output,'vectornav_binary_data.txt'), 'w') #w for write as opposed to append
        while True:
            try:
                line = ser.readline() # unsure if needed -> .decode('utf-8')
                if line:
                    # if '$VNINS' in line:
                    data = line[0:].strip() #7 is the number of chars to skip
                    if args.nothing == False:
                        print(data)
                    file.write(data+'\n')
                if line =='!':
                    ser.close()
                    break
            
            except:
                print('')
                print('user input ended logging')
                print('')
                ser.flush()
                ser.close()
                break

    else:
        print("cannot open serial port")

if __name__=='__main__':
    main()
