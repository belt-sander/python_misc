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
    arg_parser = argparse.ArgumentParser(description='evaluate tesla bicycle model')
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
    vehicleStateVelocityError = np.zeros((len(ubxInput),1))

    # data to be used in num_frames
    thetaRecursiveCalc = np.zeros(((args.num_frames),1))
    thetaRecursiveCalcGlobal = np.zeros(((args.num_frames),1))
    thetaRecursiveCalcModel = np.zeros(((args.num_frames),1))
    thetaRecursiveCalcGlobalModel = np.zeros(((args.num_frames),1))
    thetaStaticModel = np.zeros(((args.num_frames),1))
    thetaStaticModelGlobal = np.zeros(((args.num_frames),1))    

    xVariableRecursiveCalc = np.zeros(((args.num_frames),1))
    yVariableRecursiveCalc = np.zeros(((args.num_frames),1))
    xVariableRecursiveCalcModel = np.zeros(((args.num_frames),1))
    yVariableRecursiveCalcModel = np.zeros(((args.num_frames),1))
    xVariableRecursiveCalcOffset = np.zeros(((args.num_frames),1))
    yVariableRecursiveCalcOffset = np.zeros(((args.num_frames),1))
    xVariableRecursiveCalcModelOffset = np.zeros(((args.num_frames),1))
    yVariableRecursiveCalcModelOffset = np.zeros(((args.num_frames),1))
    xVariableStaticCalc = np.zeros(((args.num_frames),1))
    yVariableStaticCalc = np.zeros(((args.num_frames),1))
    xVariableStaticCalcOffset = np.zeros(((args.num_frames),1))
    yVariableStaticCalcOffset = np.zeros(((args.num_frames),1))

    abbrievatedUbxTime = np.zeros(((args.num_frames),1))
    abbreviatedUbxHeading = np.zeros(((args.num_frames),1))
    abbreviatedMotionNorthing = np.zeros(((args.num_frames),1))
    abbreviatedMotionEasting = np.zeros(((args.num_frames),1))
    abbreviatedSteerWheelAngle = np.zeros(((args.num_frames),1))
    abbreviatedSteerWheelAngleModel = np.zeros(((args.num_frames),1))    
    abbreviatedAvgAxleSpeed = np.zeros(((args.num_frames),1))
    localMotionNorthing = np.zeros(((args.num_frames),1))
    localMotionEasting = np.zeros(((args.num_frames),1))
    abbreviatedVnAccelX = np.zeros(((args.num_frames),1))
    abbreviatedVnAccelY = np.zeros(((args.num_frames),1))
    abbreviatedVnAccelZ = np.zeros(((args.num_frames),1))
    abbreviatedVehStateVelError = np.zeros(((args.num_frames),1))
    abbreviatedUbxVelocity = np.zeros(((args.num_frames),1))

    zero = np.zeros(((args.num_frames),1))

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
    vnAccelX = motecInput[:,11]
    vnAccelY = motecInput[:,12]
    vnAccelZ = motecInput[:,13]

    # visual time syncronization laying velocity from vehicle controller on top of SoG output from ublox
    userTimeOffset = 5.575 # file 2 
    # userTimeOffset = 11.8 # file 1 

    # tuning parameters 
    speedCorrection = 1.115 # (1.1 is pretty solid)
    steeringRackRatioGuess = 13.98 # guess at steering rack ratio (13.78 is pretty solid)
    steeringWheelAngleOffset = 1.05 # degrees added to reported CAN steering angle
    wheelBase = 2.95 # meters       

    for i, row in enumerate(motecInput):
        system_time_for = row[0]
        motecTow[i,:] = system_time_for + ubxTow[0] + userTimeOffset

    InterpMotecTow = np.resize(motecTow, np.size(motecTow)) # seconds
    InterpVelocity = np.interp(ubxTow, InterpMotecTow, forwardVelocity) # m/s
    InterpSteeringWheelAngle = np.interp(ubxTow, InterpMotecTow, (steeringWheelAngle + steeringWheelAngleOffset)) # degree
    InterpWheelSpeedLeftRear = np.interp(ubxTow, InterpMotecTow, wheelSpeedLeftRear) # m/s
    InterpWheelSpeedRightRear = np.interp(ubxTow, InterpMotecTow, wheelSpeedRightRear) # m/s
    InterpMotecVnYaw = np.interp(ubxTow, InterpMotecTow, vnYaw) # degree
    InterpMotecVnAccelX = np.interp(ubxTow, InterpMotecTow, vnAccelX) # m/s/s
    InterpMotecVnAccelY = np.interp(ubxTow, InterpMotecTow, vnAccelY) # m/s/s
    InterpMotecVnAccelZ = np.interp(ubxTow, InterpMotecTow, vnAccelZ) # m/s/s

    # convert ublox from LLA to UTM
    ubxProj = Proj("+proj=utm +zone=10S, +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs")
    for i, row in enumerate(ubxInput):
        gpsLatTruth = row[2]
        gpsLongTruth = row[3]
        utmNTruth,utmETruth = ubxProj(gpsLongTruth,gpsLatTruth)
        # print("truth utm:", utmNTruth, utmETruth)
        ubxNorthing[i,: ] = utmNTruth
        ubxEasting[i,: ] = utmETruth 
 
    # combine relevant data in to new column stack
    syncData = np.column_stack((ubxTow, ubxNorthing, ubxEasting, ubxHeading, InterpWheelSpeedLeftRear, InterpWheelSpeedRightRear, InterpSteeringWheelAngle, InterpMotecVnYaw, InterpMotecVnAccelX, InterpMotecVnAccelY, InterpMotecVnAccelZ, ubxVelocity))
    for i, row in enumerate(syncData):
        ubx_northing = row[1]
        ubx_easting = row[2]
        ubx_heading = row[3]
        lr_speed = row[4]
        rr_speed = row[5]
        steer_wheel_angle = row[6]
        vn_yaw = row[7]
        vn_accel_x = row[8]
        vn_accel_y = row[9]
        vn_accel_z = row[10]
        ubx_velocity = row[11]

        if (vn_yaw < 0):
            motecVnYaw[i,:] = vn_yaw + 360
        else:
            motecVnYaw[i,:] = vn_yaw

        gpsHeadingRad[i,:] = ubx_heading * np.pi/180
        avgRearAxleSpeed[i,:] = ((lr_speed + rr_speed) / 2) * speedCorrection
        steeredWheelAngle[i,:] = ((steer_wheel_angle / steeringRackRatioGuess) * -1)  * np.pi/180 # sign flip for right hand rule vehicle frame
        steeredWheelAngleModel[i,:] = (steer_wheel_angle / ((steeringRackRatioGuess * ((-avgRearAxleSpeed[i]/6)+1)) * -1))  * np.pi/180 # sign flip for right hand rule vehicle frame
        gpsRelativeMotionNorthing[i,:] = ubx_northing - ubxNorthing[0]
        gpsRelativeMotionEasting[i,:] = ubx_easting - ubxEasting[0]

        vehicleStateVelocityError[i,:] = ubx_velocity - avgRearAxleSpeed[i]

        # print('frame: ', i, ' avg rear speed: ', avgRearAxleSpeed[i], ' steered wheel angle: ', steeredWheelAngle[i], ' steer wheel angle direct: ', steer_wheel_angle)

    # combine data for vehicle model processing
    modelData = np.column_stack((ubxTow, gpsRelativeMotionNorthing, gpsRelativeMotionEasting, gpsHeadingRad, avgRearAxleSpeed, steeredWheelAngle, steeredWheelAngleModel, gpsHeadingRad, InterpMotecVnAccelX, InterpMotecVnAccelY, InterpMotecVnAccelZ, vehicleStateVelocityError, ubxVelocity))

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
                ubx_heading_rad = row[7]
                vn_accel_x = row[8]
                vn_accel_y = row[9]
                vn_accel_z = row[10]
                veh_state_velocity_error = row[11]
                ubx_velocity = row[12]                

                abbreviatedAvgAxleSpeed[t,:] = avg_axle_spd
                abbreviatedSteerWheelAngle[t,:] = rad_wheel_angle
                abbreviatedSteerWheelAngleModel[t,:] = rad_wheel_angle_model
                abbrievatedUbxTime[t,:] = ubx_time
                abbreviatedMotionNorthing[t,:] = rel_northing
                abbreviatedMotionEasting[t,:] = rel_easting
                abbreviatedVnAccelX[t,:] = vn_accel_x
                abbreviatedVnAccelY[t,:] = vn_accel_y
                abbreviatedVnAccelZ[t,:] = vn_accel_z
                abbreviatedVehStateVelError[t,:] = veh_state_velocity_error
                abbreviatedUbxVelocity[t,:] = ubx_velocity

                if (ubx_heading_rad > (270 * np.pi/180)):
                    abbreviatedUbxHeading[t,:] = ((ubx_heading_rad * -1) + (360 * np.pi/180) + (90 * np.pi/180))
                else:
                    abbreviatedUbxHeading[t,:] = (ubx_heading_rad * -1) + (90 * np.pi/180)

                thetaRecursiveCalc[t,:] = ((avg_axle_spd/wheelBase) * (np.tan(rad_wheel_angle)) * (0.1)) + thetaRecursiveCalc[t-1]
                thetaRecursiveCalcModel[t,:] = ((avg_axle_spd/wheelBase) * (np.tan(rad_wheel_angle_model)) * (0.1)) + thetaRecursiveCalc[t-1]
                thetaStaticModel[t,:] = ((avgRearAxleSpeed[args.start_frame]/wheelBase) * (np.tan(steeredWheelAngle[args.start_frame]) * (0.1)) + thetaStaticModel[t-1])
                
                # print(' rear axle speed: ', abbreviatedAvgAxleSpeed[t], ' wheel angle: ', abbreviatedSteerWheelAngle[t]*180/np.pi, ' wheel angle model: ', abbreviatedSteerWheelAngleModel[t]*180/np.pi, ' theta static model: ', thetaStaticModel[t]*180/np.pi, ' theta recur model: ', thetaRecursiveCalcModel[t]*180/np.pi, ' speed at args start: ', avgRearAxleSpeed[args.start_frame], ' sa at args start (rad): ', steeredWheelAngle[args.start_frame]*10)    

    # global correction
    thetaRecursiveCalcGlobal = thetaRecursiveCalc - gpsHeadingRad[args.start_frame] + (90 * (np.pi/180))
    thetaRecursiveCalcGlobalModel = thetaRecursiveCalcModel - gpsHeadingRad[args.start_frame] + (90 * (np.pi/180))
    thetaStaticModelGlobal = thetaStaticModel - gpsHeadingRad[args.start_frame] + (90 * (np.pi/180))
    thetaUbxHeadingGlobal = abbreviatedUbxHeading - gpsHeadingRad[args.start_frame] + (90 * (np.pi/180))
    
    # local correction
    localMotionNorthing = abbreviatedMotionNorthing - gpsRelativeMotionNorthing[args.start_frame]
    localMotionEasting = abbreviatedMotionEasting - gpsRelativeMotionEasting[args.start_frame]

    # x / y predictions
    for t in range(args.num_frames):
        xVariableRecursiveCalc[t,:] = (abbreviatedAvgAxleSpeed[t] * (np.cos(thetaRecursiveCalcGlobal[t])) * (0.1)) + (xVariableRecursiveCalc[t-1])
        yVariableRecursiveCalc[t,:] = (abbreviatedAvgAxleSpeed[t] * (np.sin(thetaRecursiveCalcGlobal[t])) * (0.1)) + (yVariableRecursiveCalc[t-1])
        xVariableRecursiveCalcModel[t,:] = (abbreviatedAvgAxleSpeed[t] * (np.cos(thetaRecursiveCalcGlobalModel[t])) * (0.1)) + (xVariableRecursiveCalcModel[t-1])
        yVariableRecursiveCalcModel[t,:] = (abbreviatedAvgAxleSpeed[t] * (np.sin(thetaRecursiveCalcGlobalModel[t])) * (0.1)) + (yVariableRecursiveCalcModel[t-1])
        xVariableStaticCalc[t,:] = (abbreviatedAvgAxleSpeed[0] * (np.cos(thetaStaticModelGlobal[t])) * (0.1)) + (xVariableStaticCalc[t-1])
        yVariableStaticCalc[t,:] = (abbreviatedAvgAxleSpeed[0] * (np.sin(thetaStaticModelGlobal[t])) * (0.1)) + (yVariableStaticCalc[t-1])        

        # print('frame: ', t, ' x: ', xVariableRecursiveCalc[t], ' y: ', yVariableRecursiveCalc[t], ' x model: ', xVariableRecursiveCalcModel[t], ' y model: ', yVariableRecursiveCalcModel[t], ' x static: ', xVariableStaticCalc[t], ' y static: ', yVariableStaticCalc[t])   

    xVariableRecursiveCalcOffset = xVariableRecursiveCalc - xVariableRecursiveCalc[0]
    yVariableRecursiveCalcOffset = yVariableRecursiveCalc - yVariableRecursiveCalc[0]
    xVariableRecursiveCalcModelOffset = xVariableRecursiveCalcModel - xVariableRecursiveCalcModel[0]
    yVariableRecursiveCalcModelOffset = yVariableRecursiveCalcModel - yVariableRecursiveCalcModel[0]
    xVariableStaticCalcOffset = xVariableStaticCalc - xVariableStaticCalc[0]
    yVariableStaticCalcOffset = yVariableStaticCalc - yVariableStaticCalc[0]

    # distance traveled
    abbreviatedVehDistanceTraveled = np.cumsum(abbreviatedAvgAxleSpeed) * 0.1
    abbreviatedUbxDistanceTraveled = np.cumsum(abbreviatedUbxVelocity) * 0.1
    abbreviatedLongitudinalError = ((abbreviatedVehDistanceTraveled - abbreviatedUbxDistanceTraveled) / abbreviatedUbxDistanceTraveled) * 100 # percent error

    # plot
    plt.figure(1)
    plt.subplot(411)
    plt.title('entire dataset')    
    plt.plot(ubxTow, InterpVelocity, color='red', label='forward vel vectornav (m/s)')
    plt.plot(ubxTow, ubxVelocity, color='blue', label='forward vel ubx (m/s)')
    plt.plot(ubxTow, avgRearAxleSpeed, label='average rear axle speed (m/s)')
    plt.legend()

    plt.subplot(412)
    plt.plot(ubxTow, steeredWheelAngle, label='front wheel angle (rad)')
    plt.legend()

    plt.subplot(413)
    # plt.plot(ubxTow, InterpWheelSpeedLeftRear, label='left rear wheel speed (m/s)')
    # plt.plot(ubxTow, InterpWheelSpeedRightRear, label='right rear wheel speed (m/s)')
    plt.plot(ubxTow, InterpSteeringWheelAngle, label='InterpSteeringWheelAngle (deg)')
    plt.legend()

    plt.subplot(414)
    plt.plot(ubxTow, motecVnYaw, label='vn heading')
    plt.plot(ubxTow, gpsHeadingRad*180/np.pi, label='ubx heading')
    plt.legend()

    plt.figure(2)
    plt.subplot(711)
    plt.title('data from sample window')    
    # plt.plot(abbrievatedUbxTime, thetaRecursiveCalcGlobalModel * 180/np.pi, label='theta global model (deg)')
    plt.plot(abbrievatedUbxTime, thetaRecursiveCalcGlobal * 180/np.pi, label='theta global (deg)')
    plt.plot(abbrievatedUbxTime, abbreviatedUbxHeading * 180/np.pi, label='ubx heading (deg)')
    plt.plot(abbrievatedUbxTime, zero, color='green')
    plt.legend()

    plt.subplot(712)
    # plt.plot(abbrievatedUbxTime, abbreviatedSteerWheelAngleModel, label='steer wheel angle model (rad)')
    plt.plot(abbrievatedUbxTime, abbreviatedSteerWheelAngle, label='steer wheel angle (rad)')
    plt.legend()

    plt.subplot(713)
    plt.plot(abbrievatedUbxTime, abbreviatedAvgAxleSpeed, label='rear axle speed (m/s)')
    plt.legend()

    plt.subplot(714)
    plt.plot(abbrievatedUbxTime, abbreviatedVnAccelX, label='forward accel (m/s/s)')
    plt.plot(abbrievatedUbxTime, abbreviatedVnAccelY, label='lateral accel (m/s/s)')
    plt.legend()

    plt.subplot(715)
    plt.plot(abbrievatedUbxTime, abbreviatedVehStateVelError, label='vehicle state velocity error (m/s)')
    plt.plot(abbrievatedUbxTime, zero)
    plt.legend()

    plt.subplot(716)
    plt.plot(abbrievatedUbxTime, abbreviatedUbxDistanceTraveled, label='ubx distance traveled (m)')
    plt.plot(abbrievatedUbxTime, abbreviatedVehDistanceTraveled, label='vehicle distance traveled (m)')
    plt.legend()

    plt.subplot(717)
    plt.plot(abbrievatedUbxTime, abbreviatedLongitudinalError, label='abbreviated long error (%)')
    plt.plot(abbrievatedUbxTime, zero, color='green')
    plt.legend()

    plt.figure(3)
    plt.grid(b=True)
    plt.title('entire dataset trajectory')
    plt.scatter(gpsRelativeMotionNorthing, gpsRelativeMotionEasting, label='relative motion (m)')
    plt.scatter(abbreviatedMotionNorthing, abbreviatedMotionEasting, s=25, c=abbrievatedUbxTime, label='relative local motion (m)')
    plt.legend()

    plt.figure(4)
    plt.grid(b=True)
    plt.title(steeringRackRatioGuess)
    plt.scatter(localMotionNorthing, localMotionEasting, s=25, c=abbrievatedUbxTime, label='local motion (m)')
    plt.scatter(xVariableRecursiveCalcOffset, yVariableRecursiveCalcOffset,  color='orange', label='dead reckoning vehicle data (m)')
    # plt.scatter(xVariableRecursiveCalcModelOffset, yVariableRecursiveCalcModelOffset, color='blue', label='prediction recursive vel model(m)')
    # plt.scatter(xVariableStaticCalcOffset, yVariableStaticCalcOffset, color='green', label='static t0 model (m)')    
    plt.legend()

    plt.show()

if __name__=='__main__':
    main()

