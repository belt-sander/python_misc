#!/usr/bin/env python

from __future__ import print_function
import rospy
from autodrive_msgs.msg import UbloxHnr
import argparse

def parse_args():
	arg_parser = argparse.ArgumentParser(description='A script designed to write the /gps/0/ublox/hnr topic to text file')
	arg_parser.add_argument('-o',  '--output', required=True, help='Desired output file name / path')
	return arg_parser.parse_args()

class Output():
	def __init__(self):
		self.args = parse_args()
		self.pose_id = 0

	def odom(self, msg):
		time_secs = msg.stamp.secs
		time_nsecs = msg.stamp.nsecs
		time = time_secs + (time_nsecs * 1e-9)
		lat = msg.lat_deg
		lon = msg.lon_deg
		heading = msg.ned_heading_deg
		self.pose_id = self.pose_id + 1	

		output_line = [str(time), str(lat), str(lon), str(heading)]
		# print(output_line)

		self.write(output_line)

	def write(self, output_to_text):			
		with open(self.args.output, 'a') as the_file:
			the_file.write(",".join(output_to_text) + '\n')

def listener():
	rospy.init_node('listener', anonymous=True)
	output = Output()
	rospy.Subscriber('/gps/0/ublox/hnr', UbloxHnr, output.odom)
	rospy.spin()

if __name__ == '__main__':
	listener()
