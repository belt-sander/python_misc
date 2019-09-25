#simple tool written to convert stupid motec gps time to posix time

#!/usr/bin/python

import time
import csv
import argparse
import datetime
import numpy as np
import matplotlib.pyplot as plt

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
    
    timeZoneAdjust = 25200 #7 hour (in seconds) time zone adjustment from UTC to PST (hack, I know)
    timeBase = datetime.datetime.strptime(args.date, '%d-%m-%Y')
    timeTuple = time.mktime(timeBase.timetuple())
    timeOffset = timeTuple - timeZoneAdjust
    
    print("")
    print("time base from date string below:")
    print(timeOffset)
    print("")

    inputData = np.genfromtxt(args.input_csv, skip_header=17, delimiter=',')
    print("data has been imported")
    print("")

    gpsGenPosix = np.zeros((len(inputData),1))
    sysTime = inputData[:,0]
    gpsTimeTest = inputData[:,77]
    gpsLat = inputData[:,19]
    gpsLong = inputData[:,20]
    avgSpd = inputData[:,1]
    brakeState = inputData[:,7]
    wheelSpeedFL = inputData[:,56]
    wheelSpeedFR = inputData[:,57]
    wheelSpeedRL = inputData[:,58]
    wheelSpeedRR = inputData[:,59]
    steerWheelAngle = inputData[:,62]
    wheelMoveState = inputData[:,66]
    brakePresF = inputData[:,67]
    brakePresR = inputData[:,68]
    avgBrakePres = brakePresR+brakePresF
    vnYaw = inputData[:,13]
    vnPitch = inputData[:,14]
    vnRoll = inputData[:,15]
    vnVelX = inputData[:,16]
    vnVelY = inputData[:,17]
    vnVelZ = inputData[:,18]
    vnAccelX = inputData[:,21]
    vnAccelY = inputData[:,22]
    vnAccelZ = inputData[:,23]
    vnGyroX = inputData[:,24]
    vnGyroY = inputData[:,25]
    vnGyroZ = inputData[:,26]
    rgLat = inputData[:,85]
    rgLong = inputData[:,86]
    wheelOdoRR = inputData[:,91]
    speedOBDResponse = inputData[:,49]

    print("iterating through data...")
    print("")
    for i, row in enumerate(inputData):
        gpsSec = row[77]
        gpsGenPosix[i,: ] = gpsSec + timeOffset
        # print("new posix time:")
        # print(gpsGenPosix)

    dataOutput = np.column_stack((gpsGenPosix,avgSpd,brakeState,wheelOdoRR,speedOBDResponse, wheelSpeedFL,wheelSpeedFR,wheelSpeedRL,wheelSpeedRR,steerWheelAngle,wheelMoveState,avgBrakePres,vnYaw,vnPitch,vnRoll,vnAccelX,vnAccelY,vnAccelZ,vnGyroX,vnGyroY,vnGyroZ,gpsLat,gpsLong,rgLat,rgLong, vnVelX, vnVelY, vnVelZ))
    np.savetxt(args.output, dataOutput, fmt='%.3f', delimiter=' ', header="# gpsGenPosix(s),avgSpd(mph),brakeState(unitless),wheelodometeryRR(mph),speedOBDResponse(mph),wheelSpeedFL(mph),wheelSpeedFR(mph),wheelSpeedRL(mph),wheelSpeedRR(mph),steerWheelAngle(deg),wheelMoveState(unitless),avgBrakePres(unitless),vnYaw(deg),vnPitch(deg),vnRoll(deg),vnAccelX(m/s/s),vnAccelY(m/s/s),vnAccelZ(m/s/s),vnGyroX(rad/s),vnGyroY(rad/s),vnGyroZ(rad/s),gps lat(dd), gps long(dd), rg lat(dd), rg long(dd), vnVelX(m/s), vnVelY(m/s), vnVelZ(m/s)", comments='')
    print("data has been exported")
    print("")


    # plt.subplot(3,1,1)
    plt.title("relevant vehicle data")
    plt.figure(1)
    # plt.plot(gpsGenPosix, wheelSpeedRR, color='gold', label='rr wheel')
    # plt.plot(gpsGenPosix, wheelSpeedRL, color='red', label='rl wheel')
    # plt.plot(gpsGenPosix, wheelSpeedFL, color='green', label='fl wheel')
    # plt.plot(gpsGenPosix, wheelMoveState, color='red', label='wheels moving state')
    plt.plot(gpsGenPosix, wheelSpeedFR, color='blue', label='fr wheel (CAN)')
    plt.plot(gpsGenPosix, wheelOdoRR, color='magenta', label='rr direct (Encoder)')
    plt.plot(gpsGenPosix, speedOBDResponse, color='cyan', label='obd speed response (OBD)')
    plt.ylabel('wheel speed (mph)')
    plt.xlabel('utc time (s)')
    plt.legend()
    # plt.ylabel('wheels moving state')

    # plt.subplot(3,1,2)
    # plt.plot(gpsGenPosix, steerWheelAngle, color='pink')
    # plt.ylabel('steering wheel angle (deg)')

    # plt.subplot(3,1,3)
    # plt.plot(gpsGenPosix, avgBrakePres, color='yellow')
    # plt.ylabel('avg brake press (unitless)')
    # plt.xlabel('utc time (s)')

    plt.figure(2)
    plt.title("vectornav / racegrade compare")
    plt.plot(rgLat,rgLong,color='red')
    plt.plot(gpsLat,gpsLong,color='green')
    plt.ylabel("longitude (dd)")
    plt.xlabel("latitude (dd)")

    plt.figure(3)
    plt.title("vectornav global vel")
    plt.plot(gpsGenPosix, vnVelX, color='red', label='north vel (m/s)')
    plt.plot(gpsGenPosix, vnVelY, color='blue', label='east vel (m/s)')
    plt.plot(gpsGenPosix, vnVelZ, color='green', label='down vel (m/s)')
    plt.xlabel('utc time (s)')
    plt.ylabel('vel (m/s)')
    plt.legend()

    plt.show()

if __name__=='__main__':
    main()

