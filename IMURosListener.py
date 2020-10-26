#!/usr/bin/env python

from __future__ import print_function
import rospy
from autodrive_msgs.msg import ImuMeasurementBatch
import struct
import time
import numpy as np
import argparse
import sys

def parse_args():
    arg_parser = argparse.ArgumentParser(description='A tool to extract IMU data from batch measurements')
    arg_parser.add_argument('-o',  '--output', required=False, help='Desired output file name / path')
    return arg_parser.parse_args()

class Output():
    def __init__(self):
        self.args = parse_args()
        self.time_data = None
        self.yaw_rate = []

    def callback(self, data):
        self.time_data = data.stamp

        for i in data.measurements:
            print(data.measurements.angular_velocity_z_rps[i])


    def write(self, time, yaw_rate):
        output_data = [str(time), str(yaw_rate)]

        with open(self.args.output, 'a') as the_file:
            the_file.write(",".join(output_data) + '\n')

def listener():
    args = parse_args()
    rospy.init_node('listener', anonymous=True)
    output = Output()
    rospy.Subscriber('/imu/0/fifo_measurement', ImuMeasurementBatch, output.callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
