#!/usr/bin/env python

from __future__ import print_function
import rospy
from autodrive_msgs.msg import VehicleControlMsg
import struct
import time
import numpy as np
import argparse
import sys

def parse_args():
    arg_parser = argparse.ArgumentParser(description='A tool to extract data from /vehicle_control_rsp (binary blob) and write to a text file')
    arg_parser.add_argument('-o',  '--output', required=True, help='Desired output file name / path')
    arg_parser.add_argument('-f',   '--field', required=True, help='Choose either (steering) or (wheel)')
    return arg_parser.parse_args()

def callback(data):
    args = parse_args()
    bytearray_data = bytearray()
    bytearray_data = data.data

    time_array_data = bytearray()
    time_array_data.append(bytearray_data[0])
    time_array_data.append(bytearray_data[1])
    time_array_data.append(bytearray_data[2])
    time_array_data.append(bytearray_data[3])
    time_array_data.append(bytearray_data[4])
    time_array_data.append(bytearray_data[5])
    time_array_data.append(bytearray_data[6])
    time_array_data.append(bytearray_data[7])                
    time_data = struct.unpack('Q', time_array_data)

    id_array_data = bytearray()
    id_array_data.append(bytearray_data[32])
    id_data = struct.unpack('B', id_array_data)

    i = 1

    if args.field == 'wheel':
        if id_data[0] == 14:    
            lf_ws_data = bytearray()
            lf_ws_data.append(bytearray_data[8])
            lf_ws_data.append(bytearray_data[9])
            lf_ws_data.append(bytearray_data[10])
            lf_ws_data.append(bytearray_data[11])
            lf_ws = struct.unpack('f', lf_ws_data)

            rf_ws_data = bytearray()
            rf_ws_data.append(bytearray_data[12])
            rf_ws_data.append(bytearray_data[13])
            rf_ws_data.append(bytearray_data[14])
            rf_ws_data.append(bytearray_data[15])
            rf_ws = struct.unpack('f', rf_ws_data)

            lr_ws_data = bytearray()
            lr_ws_data.append(bytearray_data[16])
            lr_ws_data.append(bytearray_data[17])
            lr_ws_data.append(bytearray_data[18])
            lr_ws_data.append(bytearray_data[19])
            lr_ws = struct.unpack('f', lr_ws_data)

            rr_ws_data = bytearray()
            rr_ws_data.append(bytearray_data[20])
            rr_ws_data.append(bytearray_data[21])
            rr_ws_data.append(bytearray_data[22])
            rr_ws_data.append(bytearray_data[23])
            rr_ws = struct.unpack('f', rr_ws_data)

            if (rf_ws[0] - lf_ws[0]) < 5: ### filter to try and isolate good data from "false" data
                if lr_ws[0] < 200:
                    print('time:', time_data[0], 'id:', id_data[0], ' lf (m/s):', "{0:.2f}".format(lf_ws[0]), 'rf (m/s):', "{0:.2f}".format(rf_ws[0]), 'lr (m/s):', "{0:.2f}".format(lr_ws[0]), 'rr (m/s):', "{0:.2f}".format(rr_ws[0]))            
                    
                    output_data = ["{0:.6f}".format(time_data[0] * 1e-9), "{0:.4f}".format(lf_ws[0]), "{0:.4f}".format(rf_ws[0]), "{0:.4f}".format(lr_ws[0]), "{0:.4f}".format(rr_ws[0])]
                    
                    with open(args.output, 'a') as the_file:
                        the_file.write(",".join(output_data) + '\n')

    if args.field == 'steering':
        if id_data[0] == 11:
            something_data = bytearray()
            something_data.append(bytearray_data[8])
            something_data.append(bytearray_data[9])
            something_data.append(bytearray_data[10])
            something_data.append(bytearray_data[11])
            something = struct.unpack('f', something_data)
            
            print('time:', time_data[0], 'id:', id_data[0], ' steering angle (deg):', "{0:.2f}".format(something[0]))   

            output_data = ["{0:.6f}".format(time_data[0] * 1e-9), "{0:.2f}".format(something[0])]
                    
            with open(args.output, 'a') as the_file:
                the_file.write(",".join(output_data) + '\n')        

    # else:
    #     print('incorrect field entered')
    #     sys.exit(0)


def listener():
    args = parse_args()
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('vehicle_control_rsp', VehicleControlMsg, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
