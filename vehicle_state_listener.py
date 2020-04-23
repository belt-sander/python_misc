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
    return arg_parser.parse_args()

class Output():
    def __init__(self):
        self.time_data = None
        self.id_data = None
        self.lf_ws = None
        self.rf_ws = None
        self.lr_ws = None
        self.rr_ws = None
        self.steer = None

    def callback(self, data):
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
        self.time_data = struct.unpack('Q', time_array_data)

        id_array_data = bytearray()
        id_array_data.append(bytearray_data[32])
        self.id_data = struct.unpack('B', id_array_data)

        i = 1

        if self.id_data[0] == 14:    
            lf_ws_data = bytearray()
            lf_ws_data.append(bytearray_data[8])
            lf_ws_data.append(bytearray_data[9])
            lf_ws_data.append(bytearray_data[10])
            lf_ws_data.append(bytearray_data[11])
            self.lf_ws = struct.unpack('f', lf_ws_data)

            rf_ws_data = bytearray()
            rf_ws_data.append(bytearray_data[12])
            rf_ws_data.append(bytearray_data[13])
            rf_ws_data.append(bytearray_data[14])
            rf_ws_data.append(bytearray_data[15])
            self.rf_ws = struct.unpack('f', rf_ws_data)

            lr_ws_data = bytearray()
            lr_ws_data.append(bytearray_data[16])
            lr_ws_data.append(bytearray_data[17])
            lr_ws_data.append(bytearray_data[18])
            lr_ws_data.append(bytearray_data[19])
            self.lr_ws = struct.unpack('f', lr_ws_data)

            rr_ws_data = bytearray()
            rr_ws_data.append(bytearray_data[20])
            rr_ws_data.append(bytearray_data[21])
            rr_ws_data.append(bytearray_data[22])
            rr_ws_data.append(bytearray_data[23])
            self.rr_ws = struct.unpack('f', rr_ws_data)
            self.write()


        elif self.id_data[0] == 11:
            steer_data = bytearray()
            steer_data.append(bytearray_data[8])
            steer_data.append(bytearray_data[9])
            steer_data.append(bytearray_data[10])
            steer_data.append(bytearray_data[11])
            self.steer = struct.unpack('f', steer_data)

    def write(self):
        args = parse_args()
        if self.time_data is not None:
            if (self.rf_ws[0] - self.lf_ws[0]) < 5: ### filter to try and isolate good data from "false" data
                if self.lr_ws[0] < 200: ### filter to try and isolate good data from "false" data
                    print(print('time:', self.time_data[0], 'id:', self.id_data[0], ' FL (m/s):', "{0:.2f}".format(self.lf_ws[0]), 'FR (m/s):', "{0:.2f}".format(self.rf_ws[0]), 'RL (m/s):', "{0:.2f}".format(self.lr_ws[0]), 'RR (m/s):', "{0:.2f}".format(self.rr_ws[0]), "SA (deg)", "{0:.2f}".format(self.steer[0])))         
                    output_data = ["{0:.6f}".format(self.time_data[0] * 1e-9), "{0:.4f}".format(self.lf_ws[0]), "{0:.4f}".format(self.rf_ws[0]), "{0:.4f}".format(self.lr_ws[0]), "{0:.4f}".format(self.rr_ws[0]), "{0:.2f}".format(self.steer[0])]
                    
                    with open(args.output, 'a') as the_file:
                        the_file.write(",".join(output_data) + '\n')

def listener():
    args = parse_args()
    rospy.init_node('listener', anonymous=True)
    output = Output()
    rospy.Subscriber('vehicle_control_rsp', VehicleControlMsg, output.callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
