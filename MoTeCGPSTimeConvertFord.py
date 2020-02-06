#simple tool written to convert stupid motec gps time to posix time

#!/usr/bin/python

from __future__ import print_function
import time
import calendar
import csv
import argparse
import datetime
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter

def parse_args():
    arg_parser = argparse.ArgumentParser(description='convert MoTeC i2 gps time / date to posix')

    arg_parser.add_argument('-i', '--input_csv', required=True, help='MoTeC i2 csv output')
    arg_parser.add_argument('-d', '--date', required=True, type=str, help='Date (day, month, year) that data was recorded in format <xx-xx-xxxx>')
    arg_parser.add_argument('-o', '--output', required=True, help='corrected log file output location')
    arg_parser.add_argument('-r', '--radius', required=True, type=float, help='tire radius (meters) from vehicle to convert rad/sec to m/s') ### 0.40513 for velodyne f150 king ranch
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

    inputData = np.genfromtxt(args.input_csv, skip_header=100, skip_footer=100, delimiter=',')
    print("data has been imported")
    print("")

    # novatel UTC time == inputData[:,40]
    sysTime = inputData[:,0]
    wheelSpeedFL = inputData[:,47] * args.radius # rad/sec to m/s
    wheelSpeedFR = inputData[:,46] * args.radius # rad/sec to m/s
    wheelSpeedRL = inputData[:,45] * args.radius # rad/sec to m/s
    wheelSpeedRR = inputData[:,44] * args.radius # rad/sec to m/s
    steerWheelAngle = inputData[:,18] # degree
    vnYaw = inputData[:,75]
    vnPitch = inputData[:,76]
    vnRoll = inputData[:,77]
    vnVelX = inputData[:,92]
    vnVelY = inputData[:,93]
    vnVelZ = inputData[:,94]
    vnAccelX = inputData[:,78]
    vnAccelY = inputData[:,79]
    vnAccelZ = inputData[:,80]
    vnGyroX = inputData[:,81]
    vnGyroY = inputData[:,82]
    vnGyroZ = inputData[:,83]
    vnLat = inputData[:,90]
    vnLong = inputData[:,91]
    vnSats = inputData[:,86]

    steerWheelAngleRate = np.zeros((len(inputData),1))
    brakePres = np.zeros((len(inputData),1))
    novYaw = np.zeros((len(inputData),1))
    novEastVel = np.zeros((len(inputData),1))
    novNorthVel = np.zeros((len(inputData),1))
    novUpVel = np.zeros((len(inputData),1))
    gpsVnGenPosix = np.zeros((len(inputData),1))
    gpsErrorPosix = np.zeros((len(inputData),1))

    for i, row in enumerate(inputData):
        _year = int(row[100]) + 2000
        _month = int(row[101])
        _day = int(row[102])
        _hour = int(row[103]) 
        _minute = int(row[104])
        _second = int(row[105])
        _ms = int(row[106] * 1000) ### millisecond to microsecond

        # print('year: ', _year, ' month: ', _month, ' day: ', _day, ' hour: ', _hour, ' minute: ', _minute, ' second: ', _second, ' microsecond: ', _ms)

        _time_tuple = calendar.timegm((_year, _month, _day, _hour, _minute, _second)) + _ms/1e6
        gpsVnGenPosix[i,:] = _time_tuple

    dataOutput = np.column_stack((  gpsVnGenPosix,wheelSpeedFL,wheelSpeedFR,
                                    wheelSpeedRL,wheelSpeedRR,steerWheelAngle,brakePres,
                                    vnYaw,vnPitch,vnRoll,vnAccelX,
                                    vnAccelY,vnAccelZ,vnGyroX,vnGyroY,
                                    vnGyroZ,vnLat,vnLong, vnVelX, 
                                    vnVelY, vnVelZ, novEastVel, novNorthVel, 
                                    novUpVel, gpsVnGenPosix ))
    np.savetxt(args.output, dataOutput, fmt='%.8f', delimiter=' ', header="# gpsVnPosix(s),wheelSpeedFL(m/s),wheelSpeedFR(m/s),wheelSpeedRL(m/s),wheelSpeedRR(m/s),steerWheelAngle(deg),BrakePres(unitless),vnYaw(deg),vnPitch(deg),vnRoll(deg),vnAccelX(m/s/s),vnAccelY(m/s/s),vnAccelZ(m/s/s),vnGyroX(deg/s),vnGyroY(deg/s),vnGyroZ(deg/s), vn lat(dd), vn long(dd), vnVelX(mph), vnVelY(mph), vnVelZ(mph), novEastVel(mph), novNorthVel(mph), novUpVel(mph), gps time VN(s)", comments='')

    ### dark mode
    plt.style.use('dark_background')

    ### can wheel speeds and move states
    plt.figure(1)
    plt.subplot(211)
    plt.title("velocity / speed data")
    plt.plot(gpsVnGenPosix, wheelSpeedRR, color='gold', label='rr wheel (CAN) m/s')
    plt.plot(gpsVnGenPosix, wheelSpeedRL, color='red', label='rl wheel (CAN) m/s')
    plt.plot(gpsVnGenPosix, wheelSpeedFL, color='green', label='fl wheel (CAN) m/s')
    plt.plot(gpsVnGenPosix, wheelSpeedFR, color='blue', label='fr wheel (CAN) m/s')
    plt.plot(gpsVnGenPosix, vnVelX, color='purple', label='forward velocity (m/s)')
    plt.legend()

    plt.subplot(212)
    plt.plot(gpsVnGenPosix, steerWheelAngle, label='actual steering wheel angle (deg)')
    plt.legend()
    plt.ylabel('SA (degrees)')

    ### body velocity
    plt.figure(2)
    plt.subplot(311)
    plt.title("body frame velocity")
    plt.plot(gpsVnGenPosix, vnVelX, color='red', label='forward vel (m/s)')
    plt.plot(gpsVnGenPosix, vnVelY, color='blue', label='lateral vel (m/s)')
    plt.plot(gpsVnGenPosix, vnVelZ, color='green', label='up/down vel (m/s)')
    plt.ylabel('vel (m/s)')
    plt.legend()

    plt.subplot(312)
    plt.plot(gpsVnGenPosix, vnYaw, label='vn yaw (deg)')
    plt.plot(gpsVnGenPosix, vnPitch, label='vn pitch (deg)')
    plt.plot(gpsVnGenPosix, vnRoll, label='vn roll (deg)')
    plt.legend()

    plt.subplot(313)
    plt.plot(gpsVnGenPosix, vnGyroZ, label='gyro Z (deg/sec)')
    plt.plot(gpsVnGenPosix, vnGyroY, label='gyro Y (deg/sec)')
    plt.plot(gpsVnGenPosix, vnGyroX, label='gyro X (deg/sec)')     
    plt.legend()

    plt.show()

if __name__=='__main__':
    main()

