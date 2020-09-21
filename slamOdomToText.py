#!/usr/bin/env python

from __future__ import print_function
import rospy
from nav_msgs.msg import Odometry
import argparse

def parse_args():
	arg_parser = argparse.ArgumentParser(description='A script designed to write the /lidar_rectifier/odometry topic to text file')
	arg_parser.add_argument('-o',  '--output', required=True, help='Desired output file name / path')
	return arg_parser.parse_args()

class Output():
	def __init__(self):
		self.args = parse_args()
		self.pose_id = 0

	def odom(self, msg):
		time_secs = msg.header.stamp.secs
		time_nsecs = msg.header.stamp.nsecs
		time = time_secs + (time_nsecs * 1e-9)
		x = msg.pose.pose.position.x
		y = msg.pose.pose.position.y
		z = msg.pose.pose.position.z
		qx = msg.pose.pose.orientation.x
		qy = msg.pose.pose.orientation.y
		qz = msg.pose.pose.orientation.z
		qw = msg.pose.pose.orientation.w
		self.pose_id = self.pose_id + 1	

		output_line = [	str(self.pose_id),
						str(1),
						str(self.pose_id), 
						str(x), str(y), str(z),
						str(qx), str(qy), str(qz), str(qw),
						str(time)]

		self.write(output_line)

	def write(self, output_to_text):			
		with open(self.args.output, 'a') as the_file:
			the_file.write(",".join(output_to_text) + '\n')

def listener():
	rospy.init_node('listener', anonymous=True)
	output = Output()
	rospy.Subscriber('/lidar_rectifier/odometry', Odometry, output.odom)
	rospy.spin()

if __name__ == '__main__':
	listener()
