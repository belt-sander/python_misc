#simple tool written to convert stupid motec gps time to posix time

#!/usr/bin/python

import time
import csv
import argparse
import datetime
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter

def parse_args():
    arg_parser = argparse.ArgumentParser(description='convert MoTeC i2 gps time / date to posix')

    arg_parser.add_argument('-i',
                            '--input_csv', 
                            required=True, 
                            help='MoTeC i2 csv output')
    arg_parser.add_argument('-d',
                            '--date',
                            required=True,
                            type=str,
                            help='Date (day, month, year) that data was recorded in format <xx-xx-xxxx>')
    arg_parser.add_argument('-o',
                            '--output',
                            required=True,
                            help='corrected log file output location')

    return arg_parser.parse_args()

def main():
    args = parse_args()
    
    timeZoneAdjust = 28800 #7 hour (25200 seconds) / or / 8 hour (28800 seconds) time zone adjustment from UTC to PST (hack, I know)
    timeBase = datetime.datetime.strptime(args.date, '%d-%m-%Y')
    timeTuple = time.mktime(timeBase.timetuple())
    timeOffset = timeTuple - timeZoneAdjust
    
    print("")
    print("time base from date string below:")
    print(timeOffset)
    print("")

    inputData = np.genfromtxt(args.input_csv, skip_header=100, delimiter=',')
    print("data has been imported")
    print("")

    gpsGenPosix = np.zeros((len(inputData),1))
    sysTime = inputData[:,0]
    gpsTimeTest = inputData[:,188] 
    engineSpd = inputData[:,176] 
    gpsLat = inputData[:,69]
    gpsLong = inputData[:,70]
    avgSpd = inputData[:,1]
    brakeState = inputData[:,127]
    wheelSpeedFL = inputData[:,193] # m/s
    wheelSpeedFR = inputData[:,194] # mph ughhh...
    wheelSpeedRL = inputData[:,182] # m/s
    wheelSpeedRR = inputData[:,195] # mph ughhh...
    steerWheelAngle = inputData[:,196]
    wheelMoveState = inputData[:,201]
    brakePresF = inputData[:,124]
    brakePresR = inputData[:,123]
    avgBrakePres = brakePresR+brakePresF
    vnYaw = inputData[:,63]
    vnPitch = inputData[:,64]
    vnRoll = inputData[:,65]
    vnVelX = inputData[:,66]
    vnVelY = inputData[:,67]
    vnVelZ = inputData[:,68]
    vnAccelX = inputData[:,71]
    vnAccelY = inputData[:,72]
    vnAccelZ = inputData[:,73]
    vnGyroX = inputData[:,74]
    vnGyroY = inputData[:,75]
    vnGyroZ = inputData[:,76]
    rgLat = inputData[:,206]
    rgLong = inputData[:,207]
    wheelOdoRL = inputData[:,1]
    wheelOdoRR = inputData[:,1]
    novYaw = inputData[:,114]
    novEastVel = inputData[:,227]
    novNorthVel = inputData[:,229]
    novUpVel = inputData[:,231]
    # speedOBDResponse = inputData[:,49]

    ### rotation test ###
    vnRelVelX = np.zeros((len(inputData),1))
    vnRelVelY = np.zeros((len(inputData),1))
    novRelVelY = np.zeros((len(inputData),1))
    novRelVelX = np.zeros((len(inputData),1))    
    vnSpd = np.zeros((len(inputData),1))
    novSpd = np.zeros((len(inputData),1))
    vnPitch = inputData[:, 64]#*(180/np.pi)
    vnRoll = inputData[:, 65]#*(180/np.pi)
    vnYaw = inputData[:, 63]#*(180/np.pi)    

    ### differentiation test
    avg_speed_post = np.zeros((len(inputData),1))
    diff_speed_post = np.zeros((len(inputData),1))
    filtered_accel_error = np.zeros((len(inputData),1))
    zero = np.zeros((len(inputData),1))

    ### fixing stupid units
    forWheelSpeedFR = np.zeros((len(inputData),1))
    forWheelSpeedRR = np.zeros((len(inputData),1))

    print("iterating through data...")
    print("")

    for i, row in enumerate(inputData):
        gpsSec = row[188]
        gpsGenPosix[i,:] = gpsSec + timeOffset
        # print("new posix time:")
        # print(gpsGenPosix)
        forVnYaw =  row[63]*(np.pi/180)
        forNovYaw = (row[114]-2.5)*(np.pi/180) ### added value to try and calibrate out lateral velocity error
        forVnVelX = row[66]
        forVnVelY = row[67]
        forVnVelZ = row[68]
        forNovEastVel = row[227]
        forNovNorthVel = row[229]
        forNovUpVel = row[231]
        vnRelVelX[i,:] = (np.sin(forVnYaw)*forVnVelY + np.cos(forVnYaw)*forVnVelX)
        vnRelVelY[i,:] = (np.cos(forVnYaw)*forVnVelY + (-np.sin(forVnYaw)*forVnVelX))
        novRelVelY[i,:] = (np.sin(forNovYaw)*forNovEastVel + np.cos(forNovYaw)*forNovNorthVel)
        novRelVelX[i,:] = (np.cos(forNovYaw)*forNovEastVel + (-np.sin(forNovYaw)*forNovNorthVel))
        vnSpd[i,:] = np.sqrt((forVnVelX**2)+(forVnVelY**2)+(forVnVelZ**2))
        novSpd[i,:] = np.sqrt((forNovNorthVel**2)+(forNovEastVel**2)+(forNovUpVel**2))
        avg_speed_post[i,:] = (wheelSpeedFL[i]+wheelSpeedFR[i]+wheelOdoRR[i]+wheelOdoRL[i])/4
        forWheelSpeedFR[i,:] = row[194] / 2.237 # mph to m/s
        forWheelSpeedRR[i,:] = row[195] / 2.237 # mph to m/s        

    dataOutput = np.column_stack((gpsGenPosix,avgSpd,brakeState,wheelOdoRR,wheelOdoRL,wheelSpeedFL,forWheelSpeedFR,wheelSpeedRL,forWheelSpeedRR,steerWheelAngle,wheelMoveState,avgBrakePres,vnYaw,vnPitch,vnRoll,vnAccelX,vnAccelY,vnAccelZ,vnGyroX,vnGyroY,vnGyroZ,gpsLat,gpsLong,rgLat,rgLong, vnVelX, vnVelY, vnVelZ, novEastVel, novNorthVel, novUpVel))
    np.savetxt(args.output, dataOutput, fmt='%.8f', delimiter=' ', header="# gpsGenPosix(s),avgSpd(mph),brakeState(unitless),wheelodometeryRR(mph),wheelodometeryRL(mph),wheelSpeedFL(mph),wheelSpeedFR(mph),wheelSpeedRL(mph),wheelSpeedRR(mph),steerWheelAngle(deg),wheelMoveState(unitless),avgBrakePres(unitless),vnYaw(deg),vnPitch(deg),vnRoll(deg),vnAccelX(m/s/s),vnAccelY(m/s/s),vnAccelZ(m/s/s),vnGyroX(deg/s),vnGyroY(deg/s),vnGyroZ(deg/s),gps lat(dd), gps long(dd), rg lat(dd), rg long(dd), vnVelX(mph), vnVelY(mph), vnVelZ(mph), novEastVel(mph), novNorthVel(mph), novUpVel(mph)", comments='')
    print("data has been exported")
    print("")

    diff_speed_post = np.gradient(np.reshape((avg_speed_post/2.237), len(inputData)),0.01)
    filtered_diff_speed_post = savgol_filter(diff_speed_post,111,1)
    filtered_vn_accel_x = savgol_filter(vnAccelX,111,1)
    filtered_accel_error = filtered_diff_speed_post - filtered_vn_accel_x
        
    ### dark mode
    plt.style.use('dark_background')

    ### can wheel speeds and move states
    plt.figure(1)
    plt.subplot(211)
    plt.title("velocity / speed data")
    plt.plot(gpsGenPosix, forWheelSpeedRR, color='gold', label='rr wheel (CAN) m/s')
    plt.plot(gpsGenPosix, wheelSpeedRL, color='red', label='rl wheel (CAN) m/s')
    # plt.plot(gpsGenPosix, wheelMoveState, color='red', label='wheels moving state')
    plt.plot(gpsGenPosix, wheelSpeedFL, color='green', label='fl wheel (CAN) m/s')
    plt.plot(gpsGenPosix, forWheelSpeedFR, color='blue', label='fr wheel (CAN) m/s')

    ### wheel odo
    # plt.plot(gpsGenPosix, wheelOdoRR, color='magenta', label='rr direct (Encoder)')
    # plt.plot(gpsGenPosix, wheelOdoRL, color='brown', label='rl direct (Encoder)')

    ### experiments
    # plt.plot(gpsGenPosix, vnRelVelY, color='red', label='vn rel Y (lateral vel)')
    # plt.plot(gpsGenPosix, vnRelVelX, color='orange', label='vn rel X (forward vel)')
    plt.plot(gpsGenPosix, novRelVelY, label='novatel rel Y (forward vel)')
    plt.plot(gpsGenPosix, novRelVelX, label='novatel rel X (lateral vel)')
    # plt.plot(gpsGenPosix, vnSpd, color='black', label='vn speed mag')
    # plt.plot(gpsGenPosix, novSpd, label='novatel speed mag')
    # plt.plot(gpsGenPosix, speedOBDResponse, color='cyan', label='obd speed response (OBD)')
    plt.ylabel('wheel speed (mph)')
    plt.legend()

    plt.subplot(212)
    plt.plot(gpsGenPosix, steerWheelAngle, label='actual steering wheel angle (deg)')
    plt.plot(gpsGenPosix, (steerWheelAngle/10.93), label='front wheel angle (deg)') # front wheel angle after steering ratio reduction
    plt.legend()
    plt.ylabel('SA (degrees)')
    plt.xlabel('utc time (s)')


    ### global velocity / rotated body velocity
    plt.figure(2)
    plt.subplot(311)
    plt.title("global velocity")
    plt.plot(gpsGenPosix, vnVelX, color='red', label='north vel (m/s)')
    plt.plot(gpsGenPosix, vnVelY, color='blue', label='east vel (m/s)')
    plt.plot(gpsGenPosix, vnVelZ, color='green', label='down vel (m/s)')
    plt.plot(gpsGenPosix, novEastVel, label='novatel east vel (m/s)')
    plt.plot(gpsGenPosix, novNorthVel, label='novatel north vel (m/s)')
    plt.ylabel('vel (m/s)')
    plt.legend()

    ### orientation
    plt.subplot(312)
    plt.plot(gpsGenPosix, vnYaw, label='yaw (deg)')
    # plt.plot(gpsGenPosix, vnPitch, label='pitch (deg)')
    # plt.plot(gpsGenPosix, vnRoll, label='roll (deg)')
    plt.legend()

    ### gyro z 
    plt.subplot(313)
    plt.plot(gpsGenPosix, vnGyroZ, label='gyro Z (deg/sec)')
    # plt.plot(gpsGenPosix, vnGyroY, '-o', label='gyro Y (deg/sec)')
    # plt.plot(gpsGenPosix, vnGyroX, '-o', label='gyro X (deg/sec)')     
    plt.xlabel('utc time (s)')
    plt.legend()

    # plt.figure(3)
    # plt.subplot(211)
    # plt.plot(gpsGenPosix, (avg_speed_post/2.237), label='avg speed m/s')
    # plt.legend()
    # plt.subplot(212)
    # # plt.plot(gpsGenPosix, diff_speed_post, label='deriv speed x m/s/s raw')
    # # plt.plot(gpsGenPosix, vnAccelX, label='vn accel x m/s/s raw')
    # plt.plot(gpsGenPosix, filtered_diff_speed_post, label='filtered deriv speed x m/s/s')
    # plt.plot(gpsGenPosix, filtered_vn_accel_x, label='filtered vn accel x m/s/s')
    # plt.plot(gpsGenPosix, filtered_accel_error, label='error')
    # plt.plot(gpsGenPosix, zero, label='zero')
    # plt.legend()
    # # plt.subplot(313)
    # # plt.plot(gpsGenPosix, filtered_accel_error, label='error between deriv and accel filter')
    # # plt.ylim(-2,2)
    # # plt.plot(zero)
    # plt.legend()

    plt.show()

if __name__=='__main__':
    main()

