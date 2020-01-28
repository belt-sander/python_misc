#simple tool written to convert stupid motec gps time to posix time

#!/usr/bin/python

from __future__ import print_function
import time
import csv
import argparse
import datetime
import numpy as np
import matplotlib.pyplot as plt

def parse_args():
    arg_parser = argparse.ArgumentParser(description='convert MoTeC i2 gps time / date to posix')

    arg_parser.add_argument('-t',
                            '--truthTrajectory', 
                            required=True, 
                            help='output from Inertial Explorer research format')
    arg_parser.add_argument('-c',
                            '--compareTrajectory',
                            required=True,
                            help='output from MoTeCGPSTimeConvert.py')
    arg_parser.add_argument('-to',
                            '--truthOutput',
                            required=False,
                            help='truth output trajectory file location')

    return arg_parser.parse_args()

def main():
    args = parse_args() 
    truth = np.genfromtxt(args.truthTrajectory, skip_header=40, skip_footer=40, delimiter='')
    compare = np.genfromtxt(args.compareTrajectory, skip_header=1, delimiter='')
    
    num_truth_poses = np.size(truth, 0)
    num_compare_poses = np.size(compare, 0)

    print()
    print("num truth poses: ", num_truth_poses)
    print("num compare poses: ", num_compare_poses)
    print()

    gpsTimeTruth = truth[:,0]
    gpsLatTruth = truth[:,1]
    gpsLongTruth = truth[:,2]
    gpsHeadingTruth = truth[:,13]

    gpsTimeCompare = compare[:,0]
    gpsRgLat = compare[:,17]
    gpsRgLong = compare[:,18]

    print('time: ', gpsTimeTruth)
    print('lat novatel: ', gpsLatTruth)
    print('lon novatel: ', gpsLongTruth)
    print()
    print('time: ', gpsTimeCompare)
    print('lat rg: ', gpsRgLat)
    print('lon rg: ', gpsRgLong)
    print()
    print('time error (minutes): ', (gpsTimeTruth[100]-gpsTimeCompare[100])/60)
    print()

    if args.truthOutput is not None:
        dataOutputTruth = np.column_stack((gpsTimeTruth,gpsLatTruth,gpsLongTruth,gpsHeadingTruth))
        np.savetxt(args.truthOutput, dataOutputTruth, fmt='%.9f', delimiter=',')
        np.savetxt(args.truthOutput, dataOutputTruth, fmt='%.9f', delimiter=' ', header="# gps time (s), gps latitude (dd), gps longitude (dd), heading (deg)", comments='')

    plt.subplot(1,1,1)
    plt.title('gps trajectory compare')
    plt.plot(gpsLatTruth, gpsLongTruth, color='blue', label='ground truth trajectory')
    plt.plot(gpsRgLat, gpsRgLong, color='green', label='racegrade trajectory')
    plt.ylabel("longitude (dd)")
    plt.xlabel("latitude (dd)")
    plt.legend()

    plt.show()

if __name__=='__main__':
    main()

