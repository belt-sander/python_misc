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
        self.file = open(self.args.output, 'a')
        self.time_header = None
        self.time_frame = []
        self.imu_seq = []
        self.imu_count = 0

    def callback(self, data):
        self.time_header = data.stamp

        for i in range(len(data.measurements)):
            self.imu_count += 1
            self.time_frame.append(data.measurements[i].read_timestamp_us)
            self.imu_seq.append(data.measurements[i].sequenceNo)

            output_data = [str(self.time_header), str(self.time_frame[-1]), str(self.imu_seq[-1]), str(self.imu_count)]
            self.write(output_data)


    def write(self, output_data):
        self.file.write(",".join(output_data) + '\n')

def listener():
    args = parse_args()
    rospy.init_node('listener', anonymous=True)
    output = Output()
    rospy.Subscriber('/imu/0/fifo_measurement', ImuMeasurementBatch, output.callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
