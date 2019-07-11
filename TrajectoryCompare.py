#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import argparse

'''Parse command line args and return as a container object
'''
def parse_args():
    DEFAULT_OUTPUT_DIR = ''
    arg_parser = argparse.ArgumentParser(description='Generate trajectory comparison plots')
    arg_parser.add_argument('-g',
                            '--truthTrajectory',
                            required=True,
                            help='Txt file containing the reference trajectory')
    arg_parser.add_argument('-c',
                            '--compareTrajectory',
                            required=True,
                            help='Txt file containing the query trajectory')
    return arg_parser.parse_args()


def main():
    # Parse command line args
    args = parse_args()

    # Load data to plot
    truth = np.genfromtxt(args.truthTrajectory, delimiter=',')
    compare = np.genfromtxt(args.compareTrajectory, delimiter=',')

    num_ref_poses = np.size(truth, 0)
    num_query_poses = np.size(compare, 0)
    
    print("")
    print("data imported.")
    
    gpsTimeTruth = truth[:,0]
    gpsLatTruth = truth[:,1]
    gpsLongTruth = truth[:,2]
    gpsTimeCompare = compare[:,0]
    gpsLatCompare = compare[:,1]
    gpsLongCompare = compare[:,2]

    print("")
    print("num ref poses: ", num_ref_poses)
    print("num query poses: ", num_query_poses)
    print("")

    interpLatCompare = np.interp(gpsTimeTruth,gpsTimeCompare,gpsLatCompare)
    interpLongCompare = np.interp(gpsTimeTruth,gpsTimeCompare,gpsLongCompare)
    print("interp'd latitude",interpLatCompare)
    print("")
    print("interp array size", np.size(interpLatCompare,0))
    print("")
    print("interp'd longitude",interpLongCompare)
    print("")
    print("interp array size", np.size(interpLongCompare,0))
    print("")

    errorLat = interpLatCompare - gpsLatTruth
    errorLong = interpLongCompare - gpsLongTruth

    plt.subplot(2,1,1)
    plt.plot(gpsTimeTruth, errorLat, color='red', label='lateral error deg')
    plt.plot(gpsTimeTruth, errorLong, color='blue', label='long error deg')
    plt.legend()
    plt.subplot(2,1,2)
    plt.plot(gpsLatTruth, gpsLongTruth, color='green', label='novatel')
    plt.plot(gpsLatCompare, gpsLongCompare, color='red', label='vectornav')
    plt.legend()

    plt.show()

if __name__=='__main__':
    main()