#!/usr/bin/python

import serial
import time
import csv
import argparse
import os

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
                            required=True,
                            help='output log file location')
    arg_parser.add_argument('-p',
                            '--print_output',
                            action='store_true',
                            required=False,
                            help='print terminal output from sensor')
    arg_parser.add_argument('-a',
                            '--ascii_group',
                            required=False,
                            default='$VNINS',
                            help='requested VectorNav message (default: $VNINS)')
    arg_parser.add_argument('-f',
                            '--data_freq',
                            required=False,
                            default='10',
                            #type=int,
                            help='requested VectorNav data frequency int (default: 40) (40hz)')

    return arg_parser.parse_args()

def vn_message_type():
    args = parse_args()

    if args.ascii_group =='$VNINS':
        print('you r1ght bruh')
    else:
        print('you lame AF')

def sensor_config():
    args = parse_args()
    ### set data update rate    

    freqConfigHeader = ('$VNWRG,07,').encode()
    freqValue = (args.data_freq)
    freqCRC = (',1*XX').encode()
    newLineCarRet = ('\r \n').encode()        
    return freqConfigHeader, freqValue, freqCRC, newLineCarRet

def main():
    args = parse_args()

    ser = serial.Serial()               
    ser.port = args.input_port              #serial port path
    ser.baudrate = args.baudrate            #baud rate
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

    ### data frequency from sensor. limited to 50hz unless baudrate is changed! 
    # freqConfigHeader = ('$VNWRG,07,').encode()
    # freqValue = (args.data_freq)
    # freqCRC = (',1*XX').encode()
    # newLineCarRet = ('\r \n').encode()        
    
    freqConfigHeader, freqValue, freqCRC, newLineCarRet = sensor_config()

    ser.write(freqConfigHeader)
    ser.write(freqValue)
    ser.write(freqCRC)
    ser.write(newLineCarRet)

    if ser.isOpen():
        file = open(os.path.join(args.output,'vectornav_data.txt'), 'w') #w for write as opposed to append
        while True:
            try:
                line = ser.readline() # unsure if needed -> .decode('utf-8')
                if line:
                    if '$VNINS' in line:
                        data = line[7:].strip() #.split(',')
                        if args.print_output:
                            print(data)
                        else:
                            print('data is being written to file')
                        file.write(data+'\n')
                if line =='!':
                    ser.close()
                    break
            
            except:
                print('')
                print('user input ended logging')
                print('')
                ser.close()
                break

    else:
        print("cannot open serial port")

if __name__=='__main__':
    main()
