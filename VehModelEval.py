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
    arg_parser.add_argument('-v',   '--vehicleData',                           required=True,      help='output from MoTeCGPSTimeConvert.py')
    arg_parser.add_argument('-n',   '--novatelTrajectory',                     required=True,      help='novatel output from TrajectoryGen.py')
    arg_parser.add_argument('-nss', '--numStartingFramesSkipped',   type=int,  required=True,      help='number of frames to skip at beginning of novatel data')
    arg_parser.add_argument('-numf','--numFrames',                  type=int,  required=True,      help='number of rows to use in novatel data')
    arg_parser.add_argument('-to',  '--trajectoryOutput',                      required=False,     help='time aligned data output from vehicle and novatel')
    return arg_parser.parse_args()

def main():
    args = parse_args() 
    truth = np.genfromtxt(args.novatelTrajectory, skip_header=args.numStartingFramesSkipped, delimiter=' ')
    vehicle = np.genfromtxt(args.vehicleData, skip_header=1, skip_footer=1, delimiter=' ')

    print('')
    print('data has been imported')

    print('')
    print('vehicle size: ', np.size(vehicle))
    print('')
    print('truth size: ', np.size(truth))

    myProjTruth = Proj("+proj=utm +zone=10S, +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs")

    print('')
    print('converting truth lla to UTM...')
    print('')

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

    xVar_calc_theta_a = np.zeros((args.numFrames,1))
    yVar_calc_theta_a = np.zeros((args.numFrames,1))
    xVar_gps_theta_a = np.zeros((args.numFrames,1))
    yVar_gps_theta_a = np.zeros((args.numFrames,1))

    xVar_calc_theta_static = np.zeros((args.numFrames,1))
    yVar_calc_theta_static = np.zeros((args.numFrames,1))
    xVar_calc_theta_recursive_model = np.zeros((args.numFrames,1))
    yVar_calc_theta_recursive_model = np.zeros((args.numFrames,1))    
    xVar_calc_theta_recursive_static = np.zeros((args.numFrames,1))
    yVar_calc_theta_recursive_static = np.zeros((args.numFrames,1))        
    xVar_gps_theta_recursive = np.zeros((args.numFrames,1))
    yVar_gps_theta_recursive = np.zeros((args.numFrames,1)) 

    theta_relative_static = np.zeros((args.numFrames,1))
    theta_reltaive_static_correction = np.zeros((args.numFrames,1))
    theta_relative_recursive_model = np.zeros((args.numFrames,1))
    theta_relative_recursive_static = np.zeros((args.numFrames,1))        
    theta_relative_recursive_correction = np.zeros((args.numFrames,1))
    theta_global = np.zeros((args.numFrames,1))

    phi = np.zeros((args.numFrames,1))
    rearAxleVel = np.zeros((args.numFrames,1))
    gpsNorthingRelativeTruth = np.zeros((args.numFrames,1))
    gpsEastingRelativeTruth = np.zeros((args.numFrames,1))
    gpsHeadingRelative = np.zeros((args.numFrames,1))
    gpsHeadingAbsolute = np.zeros((args.numFrames,1))
    steeringRackAngle = np.zeros((len(vehicle),1))
    steeringRackAngleModel = np.zeros((len(vehicle),1))
    steeringWheelAngleRate = np.zeros((len(vehicle),1))

    error_gps_x = np.zeros((args.numFrames,1))
    error_gps_y = np.zeros((args.numFrames,1))
    error_theta_static_x = np.zeros((args.numFrames,1))
    error_theta_static_y = np.zeros((args.numFrames,1))
    error_theta_recursive_x = np.zeros((args.numFrames,1))
    error_theta_recursive_y = np.zeros((args.numFrames,1))

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
            gpsHeadingTruth = row[3]-2.84  ### IMPORTANT: this is to offset the imperfect installation of the IMU in the Honda
            
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

    # derivative of position to get North and East velocity
    calcVelNorth = np.gradient(np.reshape(gpsNorthingTruth, args.numFrames)) * 100.0 # unsure why the 100.0 scalar (maybe because 100hz sample rate?)    
    calcVelEast = np.gradient(np.reshape(gpsEastingTruth, args.numFrames)) * 100.0 # unsure why the 100.0 scalar? (maybe because 100hz sample rate?)
  
    # calculate steering angle rate
    calcSteerWheelRate = np.gradient(np.reshape(steerWheelAngleFor, len(vehicle)))

    # time alignment / interpolation from vehicle data to novatel time 
    gpsTimeVehicleArr = np.resize(gpsTimeVehicleFor, np.size(gpsTimeVehicleFor))
    # steering wheel
    steerWheelAngleArr = np.resize(steerWheelAngleFor, np.size(steerWheelAngleFor))
    interpSteeredWheelAngle = (np.interp(gpsTimeTruthFor, gpsTimeVehicleArr, steerWheelAngleArr)) * np.pi/180
    # steering rack
    steeringRackAngleArr = np.resize(steeringRackAngleFor, np.size(steeringRackAngleFor))
    interpSteeringRackAngle = (np.interp(gpsTimeTruthFor, gpsTimeVehicleArr, steeringRackAngleArr)) * np.pi/180
    # steering angle rate
    steerWheelAngleRateArr = np.resize(calcSteerWheelRate, np.size(calcSteerWheelRate))
    interpSteerWheelAngleRate = (np.interp(gpsTimeTruthFor, gpsTimeVehicleArr, steerWheelAngleRateArr)) * np.pi/180

    # experiment speed scalar / used for resoloving error in novatel velocity vs honda reported velocity
    hondaVelCorrection = 0.98

    # avg speed
    avgSpeedArr = np.resize(avgSpeedFor, np.size(avgSpeedFor)) * hondaVelCorrection
    interpAvgSpeed = np.interp(gpsTimeTruthFor, gpsTimeVehicleArr, avgSpeedArr)
    # avg rear axle speed
    avgRearAxleSpeedArr = np.resize(avgRearAxleSpeedFor, np.size(avgRearAxleSpeedFor)) * hondaVelCorrection
    interpAvgRearAxleSpeed = np.interp(gpsTimeTruthFor, gpsTimeVehicleArr, avgRearAxleSpeedArr)

    # new array with data from novatel and vehicle for next for loop
    combinedData = np.column_stack((gpsTimeTruthFor,gpsNorthingTruth,gpsEastingTruth, calcHeadingOffset,calcVelNorth,calcVelEast,interpAvgRearAxleSpeed,interpSteeringRackAngle,interpSteeredWheelAngle, interpSteerWheelAngleRate))
    # write output to text file
    if args.trajectoryOutput is not None:
        np.savetxt(args.novatelTrajectoryOutput, dataOutputTruth, fmt='%.9f', delimiter=' ', header="# gps time (s), gps N UTM, gps E UTM, heading(rad), north velocity (m/s), east vel (m/s), rear axle speed (m/s), front wheel angle (rad), steering wheel angle (rad), steering wheel angle rate (rad/sec)", comments='')

    # theta calculation
    for i, row in enumerate(combinedData):
        # tuning parameters
        wheelBase = 2.7 # 106.3 inches
        trackWidth = 1.5621 # 61.5 inches rear  
        rackScalar = 1

        gpstimeTruthOutput = row[0]
        gpsNorthingTruthOutput = row[1] 
        gpsEastingTruthOutput = row[2] 
        calcHeadingOffsetOutput = row[3]
        calcVelNorthOutput = row[4]
        calcVelEastOutput = row[5]
        calcRearAxleVel = row[6]
        calcSteerRackAngleModel = (row[7] * ((-0.030*calcRearAxleVel)+1)) # derived function for Honda ONLY!!        
        calcSteerRackAngle = row[7] * rackScalar
        calcSteerWheelAngle = row[8]
        calcSteerWheelAngleRate = row[9]

        # output rear axle velocity and steer rack angle
        rearAxleVel[i,:] = calcRearAxleVel      
        steeringRackAngle[i,:] = calcSteerRackAngle  
        steeringRackAngleModel[i,:] = calcSteerRackAngleModel
        steeringWheelAngleRate[i,:] = calcSteerWheelAngleRate

        # novatel heading to relative
        gpsHeadingRelative[i,:] = calcHeadingOffsetOutput - calcHeadingOffset[0]

        # novatel heading absolute
        gpsHeadingAbsolute[i,:] = calcHeadingOffsetOutput

        # novatel global to body velocity
        calcVelBodyForward[i,:] = (np.sin(calcHeadingOffsetOutput)*calcVelEastOutput + np.cos(calcHeadingOffsetOutput)*calcVelNorthOutput)
        calcVelBodyLateral[i,:] = (np.cos(calcHeadingOffsetOutput)*calcVelEastOutput + (-np.sin(calcHeadingOffsetOutput)*calcVelNorthOutput))
         
        # novatel global to relative position
        gpsNorthingRelativeTruth[i,:] = gpsNorthingTruthOutput - gpsNorthingTruth[0]
        gpsEastingRelativeTruth[i,:] = gpsEastingTruthOutput - gpsEastingTruth[0]

        # theta  
        dt = (i*0.01)
        theta_relative_static[i,:] =    ((rearAxleVel[0]/wheelBase) * (np.tan(steeringRackAngle[0])) * (0.01)) + theta_relative_static[i-1] 
        theta_relative_recursive_model[i,:] = ((calcRearAxleVel/wheelBase) * (np.tan(calcSteerRackAngleModel)) * (0.01)) + theta_relative_recursive_model[i-1] #+ calcHeadingOffset[0]
        theta_relative_recursive_static[i,:] = ((calcRearAxleVel/wheelBase) * (np.tan(calcSteerRackAngle)) * (0.01)) + theta_relative_recursive_static[i-1] #+ calcHeadingOffset[0]
        theta_global[i,:] =             (rearAxleVel[0]/wheelBase) * (np.tan(steeringRackAngle[0])) * (0.01) + theta_global[i-1] + calcHeadingOffset[0]

    # theta global correction
    theta_relative_static_correction = theta_relative_static + calcHeadingOffset[0]
    theta_relative_recursive_correction = theta_relative_recursive_model + calcHeadingOffset[0]
    theta_relative_static_correction = theta_relative_recursive_static + calcHeadingOffset[0]        

    # relative position integration
    calcPosBodyForward = np.cumsum(calcVelBodyForward) * 0.01
    calcPosBodyLateral = np.cumsum(calcVelBodyLateral) * 0.01

    # prediction calculation A
    predTime_a = 0
    startSample_a = 0
    for predTime_a in range(args.numFrames):
        predTime_a + 1
        # print('pred time: ', predTime*0.01)
        xVar_gps_theta_a[predTime_a,:] = rearAxleVel[startSample_a]*(np.cos(gpsHeadingAbsolute[startSample_a])) * (0.01) + (xVar_gps_theta_a[predTime_a-1])
        yVar_gps_theta_a[predTime_a,:] = rearAxleVel[startSample_a]*(np.sin(gpsHeadingAbsolute[startSample_a])) * (0.01) + (yVar_gps_theta_a[predTime_a-1])      
        xVar_calc_theta_a[predTime_a,:] = rearAxleVel[startSample_a]*(np.cos(theta_global[startSample_a])) * (0.01) + (xVar_calc_theta_a[predTime_a-1]) 
        yVar_calc_theta_a[predTime_a,:] = rearAxleVel[startSample_a]*(np.sin(theta_global[startSample_a])) * (0.01) + (yVar_calc_theta_a[predTime_a-1])

    # prediction calculation experiment
    predTime_f = 0
    startSample_f = 0
    for predTime_f in range(args.numFrames-startSample_f):
        predTime_f + 1

        xVar_gps_theta_recursive[predTime_f,:] = (rearAxleVel[predTime_f]*(np.cos(gpsHeadingAbsolute[predTime_f])) * (0.01) + xVar_gps_theta_recursive[predTime_f-1]) 
        yVar_gps_theta_recursive[predTime_f,:] = (rearAxleVel[predTime_f]*(np.sin(gpsHeadingAbsolute[predTime_f])) * (0.01) + yVar_gps_theta_recursive[predTime_f-1]) 

        xVar_calc_theta_static[predTime_f,:] = (rearAxleVel[0]*(np.cos(theta_relative_static_correction[predTime_f-1])) * (0.01)) + (xVar_calc_theta_static[predTime_f-1])
        yVar_calc_theta_static[predTime_f,:] = (rearAxleVel[0]*(np.sin(theta_relative_static_correction[predTime_f-1])) * (0.01)) + (yVar_calc_theta_static[predTime_f-1])

        xVar_calc_theta_recursive_static[predTime_f,:] = (rearAxleVel[predTime_f]*(np.cos(theta_relative_static_correction[predTime_f])) * (0.01)) + (xVar_calc_theta_recursive_static[predTime_f-1])
        yVar_calc_theta_recursive_static[predTime_f,:] = (rearAxleVel[predTime_f]*(np.sin(theta_relative_static_correction[predTime_f])) * (0.01)) + (yVar_calc_theta_recursive_static[predTime_f-1])

        xVar_calc_theta_recursive_model[predTime_f,:] = (rearAxleVel[predTime_f]*(np.cos(theta_relative_recursive_correction[predTime_f])) * (0.01)) + (xVar_calc_theta_recursive_model[predTime_f-1])
        yVar_calc_theta_recursive_model[predTime_f,:] = (rearAxleVel[predTime_f]*(np.sin(theta_relative_recursive_correction[predTime_f])) * (0.01)) + (yVar_calc_theta_recursive_model[predTime_f-1])
        
        # print('sample: ', predTime_f, ' x_static: ', xVar_calc_theta_static[predTime_f], ' y_static: ', yVar_calc_theta_static[predTime_f], 'theta_relative_static: ', theta_relative_static[predTime_f])
        # print('sample: ', predTime_f, ' x_recursive: ', xVar_calc_theta_recursive[predTime_f], ' y_recursive: ', yVar_calc_theta_recursive[predTime_f], 'theta_relative_recursive: ', theta_relative_recursive[predTime_f])
 
    # error generation
    error_gps_x = gpsNorthingRelativeTruth - xVar_gps_theta_recursive
    error_gps_y =  gpsEastingRelativeTruth - yVar_gps_theta_recursive
    error_theta_recursive_x = gpsNorthingRelativeTruth - xVar_calc_theta_recursive_model
    error_theta_recursive_y = gpsEastingRelativeTruth - yVar_calc_theta_recursive_model
    error_theta_static_x = gpsNorthingRelativeTruth - xVar_calc_theta_static
    error_theta_static_y = gpsEastingRelativeTruth - yVar_calc_theta_static

    # plots
    plt.style.use('dark_background')

    plt.figure(1)
    plt.subplot(511)
    plt.title('error plots')
    plt.plot(gpsTimeTruthFor, error_gps_x, label='gps X error (m)')
    plt.plot(gpsTimeTruthFor, error_gps_y, label='gps Y error (m)')
    plt.plot(gpsTimeTruthFor, zero)
    plt.legend()

    plt.subplot(512)
    plt.plot(gpsTimeTruthFor, error_theta_recursive_x, label='error theta recursive X (m)')
    plt.plot(gpsTimeTruthFor, error_theta_recursive_y, label='error theta recursive Y (m)')
    plt.plot(gpsTimeTruthFor, zero)
#    plt.ylim(-15,15)
    plt.legend()

    plt.subplot(513)
    plt.plot(gpsTimeTruthFor, interpSteeredWheelAngle, label='steering wheel angle (rad)')
    plt.legend()

    plt.subplot(514)
    plt.plot(gpsTimeTruthFor, interpSteerWheelAngleRate, label='steering wheel angle rate (rad/sec)')
    plt.legend()

    plt.subplot(515)
    plt.plot(gpsTimeTruthFor, interpAvgRearAxleSpeed, label='rear axle velocity (m/s)')
    plt.legend()

    plt.figure(2)
    plt.grid(b=True)
    plt.title('global x/y predictions')
    plt.scatter(gpsNorthingRelativeTruth, gpsEastingRelativeTruth, color='green', label='novatel position (m)')
    plt.scatter(xVar_gps_theta_recursive, yVar_gps_theta_recursive, color='red', label='recrusive gps theta (m / x,y)')
    plt.scatter(xVar_calc_theta_recursive_model, yVar_calc_theta_recursive_model, color='purple', label='recursive calc theta (m / x,y)')
    plt.scatter(xVar_calc_theta_recursive_static, yVar_calc_theta_recursive_static, color='orange', label='static calc theta (m / x,y)')                  
    plt.scatter(xVar_calc_theta_static, yVar_calc_theta_static, color='blue', label='propagation from 0 (m / x,y)')  
    # plt.xlim(-225,225)
    # plt.ylim(-225,225)
    plt.legend()

    # plt.figure(3)
    # plt.subplot(311)
    # plt.plot(gpsTimeVehicleFor, pathRadius, label='path radius (inches)')
    # plt.plot(gpsTimeVehicleFor, steerWheelAngleFor, label='steer angle (deg)')
    # plt.legend()

    # plt.subplot(312)
    # plt.plot(gpsTimeVehicleFor, calcSteerWheelRate, label='steering rack angle rate (rad/sec)')
    # plt.legend()

    # plt.subplot(313)
    # plt.plot(gpsTimeVehicleFor, wheelSpeedFLFor, label='wheel speed FL')
    # plt.plot(gpsTimeVehicleFor, wheelSpeedFRFor, label='wheel speed FR')
    # plt.plot(gpsTimeVehicleFor, wheelSpeedRLFor, label='wheel speed RL')
    # plt.plot(gpsTimeVehicleFor, wheelSpeedRRFor, label='wheel speed RR')    
    # plt.legend()

    # plt.figure(3)
    # plt.subplot(111)
    # plt.plot(gpsTimeVehicleFor, vnAccelXFor, label='accel x m/s/s')
    # plt.plot(gpsTimeVehicleFor, vnAccelYFor, label='accel y m/s/s')
    # plt.legend()

    plt.show()

if __name__=='__main__':
    main()

