#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import argparse
from pyproj import Proj
from scipy.signal import savgol_filter

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
    args = parse_args()
    truth = np.genfromtxt(args.truthTrajectory, delimiter=',')
    compare = np.genfromtxt(args.compareTrajectory, delimiter=',')    
    print("")
    print("data imported.")
    
    gpsTimeTruth = truth[:,0]
    gpsTimeCompare = compare[:,0]
    
    num_truth_poses = np.size(truth, 0)
    num_compare_poses = np.size(compare, 0)
    print("")
    print("num ref poses: ", num_truth_poses)
    print("num query poses: ", num_compare_poses)

    myProjTruth = Proj("+proj=utm +zone=10S, +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs")
    myProjCompare = Proj("+proj=utm +zone=10S, +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs")    
    
    print("")
    print("converting truth lla to UTM...")
    gpsNorthingCompare = np.zeros((len(compare),1))
    gpsEastingCompare = np.zeros((len(compare),1))
    gpsNorthingTruth = np.zeros((len(truth),1))
    gpsEastingTruth = np.zeros((len(truth),1))

    for aa, row in enumerate(truth):
        gpsLatTruth = row[1]
        gpsLongTruth = row[2]
        utmNTruth,utmETruth=myProjTruth(gpsLongTruth,gpsLatTruth)
        # print("truth utm:", utmNTruth, utmETruth)
        gpsNorthingTruth[aa,: ] = utmNTruth
        gpsEastingTruth[aa,: ] = utmETruth 
    
    for bb, row in enumerate(compare):
        gpsLatCompare = row[1]
        gpsLongCompare = row[2]
        utmNCompare,utmECompare=myProjCompare(gpsLongCompare,gpsLatCompare)
        # print("compare utm:", utmNCompare, utmECompare)
        gpsNorthingCompare[bb,: ] = utmNCompare
        gpsEastingCompare[bb,: ] = utmECompare

    gpsNorthingTruthArr = np.resize(gpsNorthingTruth,np.size(gpsNorthingTruth))
    gpsEastingTruthArr = np.resize(gpsEastingTruth,np.size(gpsEastingTruth))
    gpsNorthingCompareArr = np.resize(gpsNorthingCompare,np.size(gpsNorthingCompare))
    gpsEastingCompareArr = np.resize(gpsEastingCompare,np.size(gpsEastingCompare))

    # print("")
    # print(np.shape(gpsTimeTruth))
    # print(np.shape(gpsNorthingTruthArr))
    # print(np.shape(gpsEastingTruthArr))
    # print("")
    # print(np.shape(gpsTimeCompare))
    # print(np.shape(gpsNorthingCompareArr))
    # print(np.shape(gpsEastingCompareArr))

    print("")
    interpEastingCompare = np.interp(gpsTimeTruth,gpsTimeCompare,gpsEastingCompareArr)
    interpNorthingCompare = np.interp(gpsTimeTruth,gpsTimeCompare,gpsNorthingCompareArr)

    print("interp'd easting",interpEastingCompare)
    print("")
    print("interp array size", np.size(interpEastingCompare,0))
    print("")
    print("interp'd northing",interpNorthingCompare)
    print("")
    print("interp array size", np.size(interpNorthingCompare,0))
    print("")

    errorEasting = interpEastingCompare - gpsEastingTruthArr
    errorNorthing = interpNorthingCompare - gpsNorthingTruthArr 
    errorEastingFilter = savgol_filter(errorEasting,9999,3)
    errorNorthingFilter = savgol_filter(errorNorthing,9999,3)

    plt.figure(1)
    plt.subplot(211)
    plt.title("trajectory comparison")
    # plt.plot(gpsTimeTruth, errorEasting, color='red', label='easting error')
    # plt.plot(gpsTimeTruth, errorNorthing, color='blue', label='northing error')
    plt.plot(gpsTimeTruth, errorEastingFilter, color='pink', label='easting filtered error')
    plt.plot(gpsTimeTruth, errorNorthingFilter, color='green', label='northing filtered error')
    plt.ylabel("meters of error (m)")
    plt.xlabel("utc (s)")
    # plt.ylim(-5,5)
    plt.legend()

    plt.subplot(212)
    plt.plot(gpsEastingTruthArr, gpsNorthingTruthArr, color='green', label='novatel (truth)')
    plt.plot(gpsEastingCompareArr, gpsNorthingCompareArr, color='red', label='compare (questionable)')
    plt.ylabel("utm northing (m)")
    plt.xlabel("utm easting (m)")
    plt.legend()

    plt.figure(2)
    plt.subplot(211)
    plt.title("position error histogram")
    plt.hist(errorEasting,  np.linspace(-10,10,200), color='steelblue', label='easting error')
    plt.ylabel("samples")
    plt.legend()
    # plt.xlabel("meters of error (m)")
    plt.subplot(212)
    plt.hist(errorNorthing, np.linspace(-10,10,200), color='red', label='northing error')
    plt.ylabel("samples")
    plt.xlabel("meters of error (m)")
    plt.legend()

    plt.show()

if __name__=='__main__':
    main()