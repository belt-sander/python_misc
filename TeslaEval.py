#!/usr/bin/python

# -f 75 -s 2030 (good right hand turn)


from __future__ import print_function
import time
import csv
import argparse
import datetime
import numpy as np
from pyproj import Proj
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter

def parse_args():
    arg_parser = argparse.ArgumentParser(description='convert MoTeC i2 gps time / date to posix')
    arg_parser.add_argument('-u',   '--ublox_input',                required=True,      help='ublox input file (csv) format')
    arg_parser.add_argument('-m',   '--motec_input',                required=True,      help='motec input file (csv) format')    
    arg_parser.add_argument('-f',   '--num_frames',     type=int,   required=True,      help='number of frames to process')
    arg_parser.add_argument('-s',   '--start_frame',    type=int,   required=True,      help='frame start number')
    return arg_parser.parse_args()

def main():
    args = parse_args()
    
    ubxInput = np.genfromtxt(args.ublox_input, skip_header=1, delimiter=',')
    motecInput = np.genfromtxt(args.motec_input, skip_header=300, skip_footer=300, delimiter=',')
    print("")
    print("data has been imported")
    print("")

    # data to be used later
    motecTow = np.zeros((len(motecInput),1)) # calculated TOW 
    InterpMotecTow = np.zeros((len(ubxInput),1)) # interp'd TOW
    ubxNorthing = np.zeros((len(ubxInput),1))
    ubxEasting = np.zeros((len(ubxInput),1))
    avgRearAxleSpeed = np.zeros((len(ubxInput),1))
    steeredWheelAngle = np.zeros((len(ubxInput),1))
    steeredWheelAngleModel = np.zeros((len(ubxInput),1))
    gpsHeadingRad = np.zeros((len(ubxInput),1))
    gpsRelativeMotionNorthing = np.zeros((len(ubxInput),1))
    gpsRelativeMotionEasting = np.zeros((len(ubxInput),1))
    motecVnYaw = np.zeros((len(ubxInput),1))

    # data to be used in num_frames
    thetaRecursiveCalc = np.zeros(((args.num_frames),1))
    thetaRecursiveCalcGlobal = np.zeros(((args.num_frames),1))
    thetaRecursiveCalcModel = np.zeros(((args.num_frames),1))
    thetaRecursiveCalcGlobalModel = np.zeros(((args.num_frames),1))

    xVariableRecursiveCalc = np.zeros(((args.num_frames),1))
    yVariableRecursiveCalc = np.zeros(((args.num_frames),1))
    xVariableRecursiveCalcModel = np.zeros(((args.num_frames),1))
    yVariableRecursiveCalcModel = np.zeros(((args.num_frames),1))
    xVariableRecursiveCalcOffset = np.zeros(((args.num_frames),1))
    yVariableRecursiveCalcOffset = np.zeros(((args.num_frames),1))
    xVariableRecursiveCalcModelOffset = np.zeros(((args.num_frames),1))
    yVariableRecursiveCalcModelOffset = np.zeros(((args.num_frames),1))

    abbrievatedUbxTime = np.zeros(((args.num_frames),1))
    abbreviatedMotionNorthing = np.zeros(((args.num_frames),1))
    abbreviatedMotionEasting = np.zeros(((args.num_frames),1))
    abbreviatedWheelAngle = np.zeros(((args.num_frames),1))
    abbreviatedWheelAngleModel = np.zeros(((args.num_frames),1))    
    abbreviatedAvgAxleSpeed = np.zeros(((args.num_frames),1))
    localMotionNorthing = np.zeros(((args.num_frames),1))
    localMotionEasting = np.zeros(((args.num_frames),1))


    # ubx data fields
    ubxTow = ubxInput[:,1]
    ubxVelocity = ubxInput[:,5]
    ubxHeading = ubxInput[:,4]
    
    # motec data fields
    systime = motecInput[:,0]
    forwardVelocity = motecInput[:,17]
    steeringWheelAngle = motecInput[:,243]
    wheelSpeedLeftRear = motecInput[:,185]
    wheelSpeedRightRear = motecInput[:,186]
    vnYaw = motecInput[:,3]

    # visual time syncronization laying velocity from vehicle controller on top of SoG output from ublox
    # userTimeOffset = 5.7 # file 2 
    userTimeOffset = 11.8 # file 1 
    for i, row in enumerate(motecInput):
        system_time_for = row[0]
        motecTow[i,:] = system_time_for + ubxTow[0] + userTimeOffset

    InterpMotecTow = np.resize(motecTow, np.size(motecTow)) # seconds
    InterpVelocity = np.interp(ubxTow, InterpMotecTow, forwardVelocity) # m/s
    InterpSteeringWheelAngle = np.interp(ubxTow, InterpMotecTow, steeringWheelAngle) # degree
    InterpWheelSpeedLeftRear = np.interp(ubxTow, InterpMotecTow, wheelSpeedLeftRear) # m/s
    InterpWheelSpeedRightRear = np.interp(ubxTow, InterpMotecTow, wheelSpeedRightRear) # m/s
    InterpMotecVnYaw = np.interp(ubxTow, InterpMotecTow, vnYaw) # degree

    # convert ublox from LLA to UTM
    ubxProj = Proj("+proj=utm +zone=10S, +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs")
    for i, row in enumerate(ubxInput):
        gpsLatTruth = row[2]
        gpsLongTruth = row[3]
        utmNTruth,utmETruth = ubxProj(gpsLongTruth,gpsLatTruth)
        # print("truth utm:", utmNTruth, utmETruth)
        ubxNorthing[i,: ] = utmNTruth
        ubxEasting[i,: ] = utmETruth 
 
    # tuning parameters 
    speedCorrection = 1.1
    steeringRackRatioGuess = 13.0 # guess at steering rack ratio
    wheelBase = 2.95 # meters    

    # combine relevant data in to new column stack
    syncData = np.column_stack((ubxTow, ubxNorthing, ubxEasting, ubxHeading, InterpWheelSpeedLeftRear, InterpWheelSpeedRightRear, InterpSteeringWheelAngle, InterpMotecVnYaw))
    for i, row in enumerate(syncData):
        ubx_northing = row[1]
        ubx_easting = row[2]
        ubx_heading = row[3]
        lr_speed = row[4]
        rr_speed = row[5]
        steer_wheel_angle = row[6] * np.pi/180
        vn_yaw = row[7]

        if (vn_yaw < 0):
            motecVnYaw[i,:] = vn_yaw + 360
        else:
            motecVnYaw[i,:] = vn_yaw

        gpsHeadingRad[i,:] = ubx_heading * np.pi/180
        avgRearAxleSpeed[i,:] = ((lr_speed + rr_speed) / 2) * speedCorrection
        steeredWheelAngle[i,:] = ((steer_wheel_angle / steeringRackRatioGuess) * -1) # sign flip for right hand rule vehicle frame
        steeredWheelAngleModel[i,:] = ((steer_wheel_angle / ((steeringRackRatioGuess * ((-avgRearAxleSpeed[i]/8)+1)) * -1))) # sign flip for right hand rule vehicle frame
        gpsRelativeMotionNorthing[i,:] = ubx_northing - ubxNorthing[0]
        gpsRelativeMotionEasting[i,:] = ubx_easting - ubxEasting[0]
        # print('frame: ', i, ' avg rear speed: ', avgRearAxleSpeed[i], ' steered wheel angle: ', steeredWheelAngle[i])

    # combine data for vehicle model processing
    modelData = np.column_stack((ubxTow, gpsRelativeMotionNorthing, gpsRelativeMotionEasting, gpsHeadingRad, avgRearAxleSpeed, steeredWheelAngle, steeredWheelAngleModel))

    # theta calculation
    for i, row in enumerate(modelData):
        if (i >= args.start_frame):
            if (i - (args.start_frame) < args.num_frames):
                t = i - args.start_frame    
                ubx_time = row[0]
                rel_northing = row[1]
                rel_easting = row[2]
                rad_heading = row[3]
                avg_axle_spd = row[4]
                rad_wheel_angle = row[5]
                rad_wheel_angle_model = row[6]

                abbreviatedAvgAxleSpeed[t,:] = avg_axle_spd
                abbreviatedWheelAngle[t,:] = rad_wheel_angle
                abbreviatedWheelAngleModel[t,:] = rad_wheel_angle_model
                abbrievatedUbxTime[t,:] = ubx_time
                abbreviatedMotionNorthing[t,:] = rel_northing
                abbreviatedMotionEasting[t,:] = rel_easting
                thetaRecursiveCalc[t,:] = ((avg_axle_spd/wheelBase) * (np.tan(rad_wheel_angle)) * (0.1)) + thetaRecursiveCalc[t-1]
                thetaRecursiveCalcModel[t,:] = ((avg_axle_spd/wheelBase) * (np.tan(rad_wheel_angle_model)) * (0.1)) + thetaRecursiveCalc[t-1]
                print('frame: ', t, ' rear axle speed: ', abbreviatedAvgAxleSpeed[t], ' wheel angle: ', abbreviatedWheelAngle[t]*180/np.pi, ' wheel angle model: ', abbreviatedWheelAngleModel[t]*180/np.pi)    

    # global correction
    thetaRecursiveCalcGlobal = thetaRecursiveCalc - gpsHeadingRad[args.start_frame] + (90 * (np.pi/180))
    thetaRecursiveCalcGlobalModel = thetaRecursiveCalcModel - gpsHeadingRad[args.start_frame] + (90 * (np.pi/180))
    
    # local correction
    localMotionNorthing = abbreviatedMotionNorthing - gpsRelativeMotionNorthing[args.start_frame]
    localMotionEasting = abbreviatedMotionEasting - gpsRelativeMotionEasting[args.start_frame]

    # x / y predictions
    for t in range(args.num_frames):
        xVariableRecursiveCalc[t,:] = (abbreviatedAvgAxleSpeed[t] * (np.cos(thetaRecursiveCalcGlobal[t])) * (0.1)) + (xVariableRecursiveCalc[t-1])
        yVariableRecursiveCalc[t,:] = (abbreviatedAvgAxleSpeed[t] * (np.sin(thetaRecursiveCalcGlobal[t])) * (0.1)) + (yVariableRecursiveCalc[t-1])
        xVariableRecursiveCalcModel[t,:] = (abbreviatedAvgAxleSpeed[t] * (np.cos(thetaRecursiveCalcGlobalModel[t])) * (0.1)) + (xVariableRecursiveCalcModel[t-1])
        yVariableRecursiveCalcModel[t,:] = (abbreviatedAvgAxleSpeed[t] * (np.sin(thetaRecursiveCalcGlobalModel[t])) * (0.1)) + (yVariableRecursiveCalcModel[t-1])
        # print('frame: ', t, ' x: ', xVariableRecursiveCalc[t], ' y: ', yVariableRecursiveCalc[t], ' x model: ', xVariableRecursiveCalcModel[t], ' y model: ', yVariableRecursiveCalcModel[t])   

    xVariableRecursiveCalcOffset = xVariableRecursiveCalc - xVariableRecursiveCalc[0]
    yVariableRecursiveCalcOffset = yVariableRecursiveCalc - yVariableRecursiveCalc[0]
    xVariableRecursiveCalcModelOffset = xVariableRecursiveCalcModel - xVariableRecursiveCalcModel[0]
    yVariableRecursiveCalcModelOffset = yVariableRecursiveCalcModel - yVariableRecursiveCalcModel[0]


    # plot
    plt.figure(1)
    plt.subplot(411)
    plt.plot(ubxTow, InterpVelocity, color='red', label='forward vel vectornav (m/s)')
    plt.plot(ubxTow, ubxVelocity, color='blue', label='forward vel ubx (m/s)')
    plt.plot(ubxTow, avgRearAxleSpeed, label='average rear axle speed (m/s)')
    plt.legend()

    plt.subplot(412)
    plt.plot(ubxTow, InterpSteeringWheelAngle, label='steering wheel angle (deg)')
    plt.legend()

    plt.subplot(413)
    plt.plot(ubxTow, InterpWheelSpeedLeftRear, label='left rear wheel speed (m/s)')
    plt.plot(ubxTow, InterpWheelSpeedRightRear, label='right rear wheel speed (m/s)')
    plt.legend()

    plt.subplot(414)
    plt.plot(ubxTow, motecVnYaw, label='vn heading')
    plt.plot(ubxTow, gpsHeadingRad*180/np.pi, label='ubx heading')
    plt.legend()

    plt.figure(2)
    plt.subplot(311)
    plt.plot(abbrievatedUbxTime, abbreviatedWheelAngle, label='wheel angle (rad)')
    plt.plot(abbrievatedUbxTime, abbreviatedWheelAngleModel, label='wheel angle model (rad)')
    plt.legend()

    plt.subplot(312)
    plt.plot(abbrievatedUbxTime, thetaRecursiveCalcGlobal, label='theta global (rad)')
    plt.plot(abbrievatedUbxTime, thetaRecursiveCalcGlobalModel, label='theta global model (rad)')
    plt.legend()

    plt.subplot(313)
    plt.plot(abbrievatedUbxTime, abbreviatedAvgAxleSpeed, label='rear axle speed (m/s)')
    plt.legend()

    plt.figure(3)
    plt.grid(b=True)
    plt.scatter(gpsRelativeMotionNorthing, gpsRelativeMotionEasting, label='relative motion (m)')
    plt.scatter(abbreviatedMotionNorthing, abbreviatedMotionEasting, s=25, c=abbrievatedUbxTime, label='relative local motion (m)')
    plt.legend()

    plt.figure(4)
    plt.grid(b=True)    
    plt.scatter(localMotionNorthing, localMotionEasting, s=25, c=abbrievatedUbxTime, label='local motion (m)')
    plt.scatter(xVariableRecursiveCalcOffset, yVariableRecursiveCalcOffset, color='orange', label='prediction recursive static ratio (m)')
    plt.scatter(xVariableRecursiveCalcModelOffset, yVariableRecursiveCalcModelOffset, color='blue', label='prediction recursive vel model(m)')    
    plt.legend()

    plt.show()

if __name__=='__main__':
    main()

