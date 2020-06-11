#!/usr/bin/env python

from __future__ import print_function
import rospy
from autodrive_msgs.msg import CanFrameValueList
import time
import numpy as np
import argparse

def parse_args():
    arg_parser = argparse.ArgumentParser(description='A tool to extract data from /can_value_list topic')
    arg_parser.add_argument('-o',  '--output', required=True, help='Desired output file name / path')
    return arg_parser.parse_args()

class Output():
    def __init__(self):
        self.time_data_secs = None
        self.time_data_nsecs = None
        self.time_stamp = None
        self.id = None

        self.time_list = []

    def callback(self, data):
        args = parse_args()
        self.time_stamp = data.stamp
        self.time_data_secs = data.stamp.secs
        self.time_data_nsecs = data.stamp.nsecs
        self.id = data.can_id

        ### 0x217 (535) == wheel speed CAN message
        if self.id == 535:
            self.time_list.append(self.time_data_secs + (self.time_data_nsecs * 1e-9))

            for i in range(len(self.time_list)):
                output_data = ['{:6}'.format(self.time_stamp), '{:f}'.format(self.time_list[i] - self.time_list[i-1])]
                print(output_data)
                with open(args.output, 'a') as file:
                    file.write(",".join(output_data) + '\n')

def listener():
    args = parse_args()
    rospy.init_node('listener', anonymous=True)
    output = Output()
    rospy.Subscriber('/can/0/value_list', CanFrameValueList, output.callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
