#simple tool written to convert stupid motec gps time to posix time

#!/usr/bin/python

from __future__ import print_function
import time
import csv
import argparse
import datetime
import numpy as np
import matplotlib.pyplot as plt
from pyproj import Proj
from scipy.signal import savgol_filter

def parse_args():
    arg_parser = argparse.ArgumentParser(description='vehicle prediction experiment')
    arg_parser.add_argument('-n',   '--novatelTrajectory',          required=True,  help='novatel output from TrajectoryGen.py')
    arg_parser.add_argument('-no',  '--novatelTrajectoryOutput',    required=True,  help='novatel utm output (used internally)')
    arg_parser.add_argument('-ns',  '--numStartingFramesSkipped', type=int,  required=True,  help='number of frames to skip at beginning of novatel data')
    arg_parser.add_argument('-num', '--numFrames',      type=int,   required=False, help='number of rows to use in novatel data')
    arg_parser.add_argument('-v',   '--vehicleData',        required=True,  help='output from MoTeCGPSTimeConvert.py')
    return arg_parser.parse_args()


def main():
    args = parse_args() 
    truth = np.genfromtxt(args.novatelTrajectory, skip_header=args.numStartingFramesSkipped, delimiter=' ')
    vehicle = np.genfromtxt(args.vehicleData, skip_header=1, delimiter=' ')

    print("")
    print("data has been imported")

    myProjTruth = Proj("+proj=utm +zone=10S, +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs")

    print("")
    print("converting truth lla to UTM...")

    zero = np.zeros((args.numFrames,1))
    gpsNorthingTruth = np.zeros((args.numFrames,1))
    gpsEastingTruth = np.zeros((args.numFrames,1))
    gpsTimeTruthFor = np.zeros((args.numFrames,1))
    gpsHeadingTruthFor = np.zeros((args.numFrames,1))
    calcVelNorth = np.zeros((args.numFrames,1))
    calcVelEast = np.zeros((args.numFrames,1))
    calcVelNorthFiltered = np.zeros((args.numFrames,1))
    calcVelEastFiltered = np.zeros((args.numFrames,1))
    calcVelBodyForward = np.zeros((args.numFrames,1))
    calcVelBodyLateral = np.zeros((args.numFrames,1))
    calcHeadingOffset = np.zeros((args.numFrames,1))
    calcPosBodyForward = np.zeros((args.numFrames,1))
    calcPosBodyLateral = np.zeros((args.numFrames,1))

    gpsTimeVehicleFor = np.zeros((len(vehicle),1))
    steerWheelAngleFor = np.zeros((len(vehicle),1))
    steeringRackAngleFor = np.zeros((len(vehicle),1))
    vnAccelXFor = np.zeros((len(vehicle),1))
    vnAccelYFor = np.zeros((len(vehicle),1))
    avgSpeedFor = np.zeros((len(vehicle),1))
    avgRearAxleSpeedFor = np.zeros((len(vehicle),1))
    pathRadius = np.zeros((len(vehicle),1))
    wheelBaseFor = np.zeros((len(vehicle),1))
    trackWidthFor = np.zeros((len(vehicle),1))
    wheelSpeedFLFor = np.zeros((len(vehicle),1))
    wheelSpeedFRFor = np.zeros((len(vehicle),1))
    wheelSpeedRLFor = np.zeros((len(vehicle),1))
    wheelSpeedRRFor = np.zeros((len(vehicle),1))

    gpsTimeTruth = truth[:,0]
    gpsLatTruth = truth[:,1]
    gpsLongTruth = truth[:,2]
    gpsHeadingTruth = truth[:,3]

    # parse novatel data
    for aa, row in enumerate(truth):
        if aa <= (args.numFrames - 1):
            gpsTimeTruth = row[0]
            gpsLatTruth = row[1]
            gpsLongTruth = row[2]
            gpsHeadingTruth = row[3]-2.75 ### IMPORTANT: this is to offset the imperfect installation of the IMU in the Honda
            
            # convert from -180 to 180 to 0 to 360
            if gpsHeadingTruth < 0:
                calcHeadingOffset[aa,: ] = (gpsHeadingTruth + 360) * np.pi/180
            else:
                calcHeadingOffset[aa,: ] = (gpsHeadingTruth) * np.pi/180

            utmETruth,utmNTruth = myProjTruth(gpsLongTruth,gpsLatTruth)
            gpsTimeTruthFor[aa,: ] = gpsTimeTruth
            gpsNorthingTruth[aa,: ] = utmNTruth
            gpsEastingTruth[aa,: ] = utmETruth
            gpsHeadingTruthFor[aa,: ] = gpsHeadingTruth

    # derivative of position to get North and East velocity
    calcVelNorth = np.gradient(np.reshape(gpsNorthingTruth, args.numFrames)) * 100.0 # unsure why the 100.0 scalar (maybe because 100hz sample rate?)    
    calcVelEast = np.gradient(np.reshape(gpsEastingTruth, args.numFrames)) * 100.0 # unsure why the 100.0 scalar? (maybe because 100hz sample rate?)
    calcVelNorthFiltered = savgol_filter(calcVelNorth, 11, 1)
    calcVelEastFiltered = savgol_filter(calcVelEast, 11, 1)

    # parse vehicle data
    for i, row in enumerate(vehicle):
        gpsTimeVehicle = row[0]
        steerWheelAngle = row[9]
        vnAccelX = row[19]
        vnAccelY = row[20]
        spdFL = row[5]
        spdFR = row[6]
        spdRL = row[7]
        spdRR = row[8]
        wheelBase = 106.3 # inches
        trackWidth = 61.5 # inches rear

        vnAccelXFor[i,:] = vnAccelX
        vnAccelYFor[i,:] = vnAccelY
        avgSpeedFor[i,:] = (spdFL+spdRR+spdFR+spdRL)/4 # speed at COG
        avgRearAxleSpeedFor[i,:] = (spdRR+spdRL)/2 # speed at center of rear axle
        wheelBaseFor[i,:] = wheelBase
        trackWidthFor[i,:] = trackWidth 
        gpsTimeVehicleFor[i,:] = gpsTimeVehicle
        steerWheelAngleFor[i,:] = steerWheelAngle
        wheelSpeedFLFor[i,:] = spdFL
        wheelSpeedFRFor[i,:] = spdFR
        wheelSpeedRLFor[i,:] = spdRL
        wheelSpeedRRFor[i,:] = spdRR


        # vehicle model calculations
        steeringRackAngleFor[i,:] = steerWheelAngle / 10.98 # steering ratio specified by Honda

        # tan(phi) = l/r 
        # this produces discontinuities around 0 and isn't super easy to use
        # pathRadius[i,:] = (wheelBase / np.tan((steerWheelAngle / 10.98)*np.pi/180))

    ### time alignment / interpolation from vehicle data to novatel time 
    gpsTimeVehicleArr = np.resize(gpsTimeVehicleFor, np.size(gpsTimeVehicleFor))
    # steering wheel
    steerWheelAngleArr = np.resize(steerWheelAngleFor, np.size(steerWheelAngleFor))
    interpSteeredWheelAngle = (np.interp(gpsTimeTruthFor, gpsTimeVehicleArr, steerWheelAngleArr)) * np.pi/180
    # steering rack
    steeringRackAngleArr = np.resize(steeringRackAngleFor, np.size(steeringRackAngleFor))
    interpSteeringRackAngle = (np.interp(gpsTimeTruthFor, gpsTimeVehicleArr, steeringRackAngleArr)) * np.pi/180
    # avg speed
    avgSpeedArr = np.resize(avgSpeedFor, np.size(avgSpeedFor))
    interpAvgSpeed = np.interp(gpsTimeTruthFor, gpsTimeVehicleArr, avgSpeedArr)
    # avg rear axle speed
    avgRearAxleSpeedArr = np.resize(avgRearAxleSpeedFor, np.size(avgRearAxleSpeedFor))
    interpAvgRearAxleSpeed = np.interp(gpsTimeTruthFor, gpsTimeVehicleArr, avgRearAxleSpeedArr)


    # write output to text file
    if args.novatelTrajectoryOutput is not None:
        dataOutputTruth = np.column_stack((gpsTimeTruthFor,gpsNorthingTruth,gpsEastingTruth, calcHeadingOffset,calcVelNorth,calcVelEast))
        np.savetxt(args.novatelTrajectoryOutput, dataOutputTruth, fmt='%.9f', delimiter=' ', header="# gps time (s), gps N UTM, gps E UTM, heading(rad), north velocity (m/s), east vel (m/s)", comments='')

    # re-open output text file and parse
    truthConverted = np.genfromtxt(args.novatelTrajectoryOutput, skip_header=1, delimiter=' ')

    # ugly method of re-parsing converted data that was saved above
    for i, row in enumerate(truthConverted):
        gpstimeTruthOutput = row[0]
        gpsNorthingTruthOutput = row[1]
        gpsEastingTruthOutput = row[2]
        calcHeadingOffsetOutput = row[3]
        calcVelNorthOutput = row[4]
        calcVelEastOutput = row[5]
        calcVelBodyForward[i,:] = (np.sin(calcHeadingOffsetOutput)*calcVelEastOutput + np.cos(calcHeadingOffsetOutput)*calcVelNorthOutput)
        calcVelBodyLateral[i,:] = (np.cos(calcHeadingOffsetOutput)*calcVelEastOutput + (-np.sin(calcHeadingOffsetOutput)*calcVelNorthOutput))
 
        # failed experiment
        # calcPosBodyForward[i,:] = (np.sin(calcHeadingOffsetOutput)*gpsEastingTruthOutput + np.cos(calcHeadingOffsetOutput)*gpsNorthingTruthOutput)
        # calcPosBodyLateral[i,:] = (np.cos(calcHeadingOffsetOutput)*gpsEastingTruthOutput + (-np.sin(calcHeadingOffsetOutput)*gpsNorthingTruthOutput))


    print("")
    print("data has been exported")
    print("")

    plt.figure(1)
    plt.subplot(511)
    plt.title('model tests')
    plt.plot(gpsNorthingTruth, gpsEastingTruth, label='position utm (m)')
    plt.legend()

    plt.subplot(512)
    plt.plot(gpsTimeTruthFor, calcVelBodyForward, label='forward vel (m/s)')
    plt.plot(gpsTimeTruthFor, calcVelBodyLateral, label='lateral vel (m/s)')
    plt.plot(gpsTimeTruthFor, interpAvgSpeed, label='avg COG speed (m/s)')
    plt.plot(gpsTimeTruthFor, interpAvgRearAxleSpeed, label='avg rear axle speed (m/s)')
    plt.plot(gpsTimeTruthFor, zero, label='zero')
    plt.legend()

    plt.subplot(513)
    plt.plot(gpsTimeTruthFor, calcVelBodyLateral, label='lateral vel (m/s)')
    plt.plot(gpsTimeTruthFor, zero, label='zero')
    plt.legend()

    plt.subplot(514)
    plt.plot(gpsTimeTruthFor, calcHeadingOffset, label='heading (rad)')
    plt.plot(gpsTimeTruthFor, interpSteeredWheelAngle, label='vehicle steer angle (rad)')
    plt.legend()

    plt.subplot(515)
    plt.plot(gpsTimeTruthFor, calcVelNorth, label = 'north vel (m/s)')
    plt.plot(gpsTimeTruthFor, calcVelEast, label = 'east vel (m/s)')
    plt.legend()

    plt.figure(2)
    plt.subplot(311)
    plt.plot(gpsTimeVehicleFor, pathRadius, label='path radius (inches)')
    plt.plot(gpsTimeVehicleFor, steerWheelAngleFor, label='steer angle (deg)')
    # plt.ylim(-500, 500)
    plt.legend()

    plt.subplot(312)
    plt.plot(gpsTimeVehicleFor, steeringRackAngleFor, label='steering rack angle (deg)')
    plt.legend()

    plt.subplot(313)
    plt.plot(gpsTimeVehicleFor, wheelSpeedFLFor, label='wheel speed FL')
    plt.plot(gpsTimeVehicleFor, wheelSpeedFRFor, label='wheel speed FR')
    plt.plot(gpsTimeVehicleFor, wheelSpeedRLFor, label='wheel speed RL')
    plt.plot(gpsTimeVehicleFor, wheelSpeedRRFor, label='wheel speed RR')    
    plt.legend()

    # plt.figure(3)
    # plt.subplot(311)
    # plt.plot(gpsTimeVehicleFor, avgSpeedFor, label='avg speed vehicle')
    # plt.legend()

    # plt.subplot(312)
    # plt.plot(gpsTimeVehicleFor, steerWheelAngleFor, label='steering wheel angle')
    # plt.legend()

    # plt.subplot(313)
    # plt.plot(gpsTimeVehicleFor, vnAccelXFor, label='accel x m/s/s')
    # plt.plot(gpsTimeVehicleFor, vnAccelYFor, label='accel y m/s/s')
    # plt.legend()

    plt.show()

if __name__=='__main__':
    main()

