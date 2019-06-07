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
    arg_parser.add_argument('-d',
                            '--data_header',
                            required=False,
                            default='$VNINS',
                            help='requested VectorNav message (default: $VNINS)')
    return arg_parser.parse_args()

def vn_message_type():


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

    except Exception, e:
        print "error open serial port: " + str(e)
        exit()

        ser.flush()

    if ser.isOpen():
        while True:
            try:
                line = ser.readline() # unsure if needed -> .decode('utf-8')
                if line:
                    if '$VNINS' in line:
                        data = line[10:].strip() #.split(',')
                        if args.print_output:
                            print(data)
                        else:
                            print('data is being written to file')
                        with open(os.path.join(args.output,'vectornav_data.txt'), 'a') as d: #w for write as opposed to append
                            d.write(data+'\n')
                if line =='!':
                    ser.close()
                    break
            
            except:
                print('Keyboard Interrupt')
                ser.close()
                break

    else:
        print("Can not open serial port")

if __name__=='__main__':
    main()
