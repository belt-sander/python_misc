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
    arg_parser.add_argument('-n', '--fake_novatel', required=True, help='generated "novatel" output from vehicle state data')
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

    inputData = np.genfromtxt(args.input_csv, skip_header=150, skip_footer=100, delimiter=',')
    print("data has been imported")
    print("")

    # novatel UTC time == inputData[:,40]
    sysTime = inputData[:,0]
    gpsTimeTest = inputData[:,72] 
    avgSpd = inputData[:,1] # m/s
    wheelSpeedFL = inputData[:,107] # m/s
    wheelSpeedFR = inputData[:,108] # m/s
    wheelSpeedRL = inputData[:,98] # m/s
    wheelSpeedRR = inputData[:,109] # m/s
    steerWheelAngleRate = inputData[:,111]
    steerWheelAngle = inputData[:,110]
    brakePres = inputData[:,126]
    vnYaw = inputData[:,27]
    vnPitch = inputData[:,28]
    vnRoll = inputData[:,29]
    vnVelX = inputData[:,44]
    vnVelY = inputData[:,45]
    vnVelZ = inputData[:,46]
    vnAccelX = inputData[:,30]
    vnAccelY = inputData[:,31]
    vnAccelZ = inputData[:,32]
    vnGyroX = inputData[:,33]
    vnGyroY = inputData[:,34]
    vnGyroZ = inputData[:,35]
    vnLat = inputData[:,42]
    vnLong = inputData[:,43]
    vnSats = inputData[:,38]
    novYaw = inputData[:,19]
    novEastVel = inputData[:,131] # mph
    novNorthVel = inputData[:,133] # mph
    novUpVel = inputData[:,135] # mph

    ### rotation test ###
    novRelVelY = np.zeros((len(inputData),1))
    novRelVelX = np.zeros((len(inputData),1))    
    vnSpd = np.zeros((len(inputData),1))
    novSpd = np.zeros((len(inputData),1))
    zero = np.zeros((len(inputData),1))

    print('iterating through data...')
    print('')

    gpsGenPosix = np.zeros((len(inputData),1))
    gpsVnGenPosix = np.zeros((len(inputData),1))
    gpsErrorPosix = np.zeros((len(inputData),1))

    for i, row in enumerate(inputData):
        # gpsSec = row[87] # rg UTC
        gpsSec = row[72] # novatel UTC
        gpsGenPosix[i,:] = gpsSec + timeOffset ### ADDED TO COMPENSATE FOR IMPERFECT GPS TIME FROM NOVATEL VIA CAN

        _year = int(row[52])
        _month = int(row[53])
        _day = int(row[54])
        _hour = int(row[55]) 
        _minute = int(row[56])
        _second = int(row[57])
        _ms = int(row[58] * 1000) ### millisecond to microsecond

        # print('year: ', _year, ' month: ', _month, ' day: ', _day, ' hour: ', _hour, ' minute: ', _minute, ' second: ', _second, ' microsecond: ', _ms)

        _time_tuple = calendar.timegm((_year, _month, _day, _hour, _minute, _second)) + _ms/1e6
        gpsVnGenPosix[i,:] = _time_tuple
        gpsErrorPosix[i,:] = gpsVnGenPosix[i] - gpsGenPosix[i]
        # print('time error: ', gpsErrorPosix[i])

    gpsGenPosixCorrected = np.zeros((len(inputData),1))
    gpsGenPosixCorrected = gpsGenPosix + np.mean(gpsErrorPosix)

    dataOutput = np.column_stack((  gpsGenPosixCorrected,wheelSpeedFL,wheelSpeedFR,
                                    wheelSpeedRL,wheelSpeedRR,steerWheelAngle,brakePres,
                                    vnYaw,vnPitch,vnRoll,vnAccelX,
                                    vnAccelY,vnAccelZ,vnGyroX,vnGyroY,
                                    vnGyroZ,vnLat,vnLong, vnVelX, 
                                    vnVelY, vnVelZ, novEastVel, novNorthVel, 
                                    novUpVel, gpsVnGenPosix ))
    np.savetxt(args.output, dataOutput, fmt='%.8f', delimiter=' ', header="# gpsGenPosixCorrected(s),wheelSpeedFL(m/s),wheelSpeedFR(m/s),wheelSpeedRL(m/s),wheelSpeedRR(m/s),steerWheelAngle(deg),BrakePres(unitless),vnYaw(deg),vnPitch(deg),vnRoll(deg),vnAccelX(m/s/s),vnAccelY(m/s/s),vnAccelZ(m/s/s),vnGyroX(deg/s),vnGyroY(deg/s),vnGyroZ(deg/s), vn lat(dd), vn long(dd), vnVelX(mph), vnVelY(mph), vnVelZ(mph), novEastVel(mph), novNorthVel(mph), novUpVel(mph), gps time VN(s)", comments='')

    dataOutputFakeNovatel = np.column_stack((   gpsVnGenPosix, vnLat, vnLong, zero,
                                                vnAccelX, vnAccelY, vnAccelZ, vnGyroX * 180/np.pi,
                                                vnGyroY * 180/np.pi, (-vnGyroZ * 180/np.pi), zero, zero, 
                                                zero, vnYaw, vnPitch, vnRoll, 
                                                zero, zero, zero, vnVelX, 
                                                vnVelY, vnVelZ))
    np.savetxt(args.fake_novatel, dataOutputFakeNovatel, fmt='%.8f', delimiter=' ', header='#gpsVnGenPosix, vnLat, vnLong, zero, vnAccelX, vnAccelY, vnAccelZ, vnGyroX, vnGyroY, vnGyroZ, zero, zero, zero, vnYaw, vnPitch, vnRoll, zero, zero, zero, vnVelX, vnVelY, vnVelZ')

    print('data has been exported')
    print('')

    print('gps time error mean: ', np.mean(gpsErrorPosix))
    print('')

    ### dark mode
    plt.style.use('dark_background')

    ### can wheel speeds and move states
    plt.figure(1)
    plt.subplot(211)
    plt.title("velocity / speed data")
    plt.plot(gpsGenPosix, wheelSpeedRR, color='gold', label='rr wheel (CAN) m/s')
    plt.plot(gpsGenPosix, wheelSpeedRL, color='red', label='rl wheel (CAN) m/s')
    plt.plot(gpsGenPosix, wheelSpeedFL, color='green', label='fl wheel (CAN) m/s')
    plt.plot(gpsGenPosix, wheelSpeedFR, color='blue', label='fr wheel (CAN) m/s')
    plt.plot(gpsGenPosix, vnVelX, color='purple', label='forward velocity (m/s)')
    plt.legend()

    plt.subplot(212)
    plt.plot(gpsGenPosix, steerWheelAngle, label='actual steering wheel angle (deg)')
    plt.legend()
    plt.ylabel('SA (degrees)')
    plt.xlabel('novatel utc time (s)')

    ### global velocity / rotated body velocity
    plt.figure(2)
    plt.subplot(311)
    plt.title("body frame velocity")
    plt.plot(gpsGenPosix, vnVelX, color='red', label='forward vel (m/s)')
    plt.plot(gpsGenPosix, vnVelY, color='blue', label='lateral vel (m/s)')
    plt.plot(gpsGenPosix, vnVelZ, color='green', label='up/down vel (m/s)')
    plt.ylabel('vel (m/s)')
    plt.legend()

    plt.subplot(312)
    plt.plot(gpsGenPosix, vnYaw, label='vn yaw (deg)')
    plt.plot(gpsGenPosix, vnPitch, label='vn pitch (deg)')
    plt.plot(gpsGenPosix, vnRoll, label='vn roll (deg)')
    plt.legend()

    plt.subplot(313)
    plt.plot(gpsGenPosix, vnGyroZ, label='gyro Z (deg/sec)')
    plt.plot(gpsGenPosix, vnGyroY, label='gyro Y (deg/sec)')
    plt.plot(gpsGenPosix, vnGyroX, label='gyro X (deg/sec)')     
    plt.xlabel('novatel utc time (s)')
    plt.legend()

    fig1, (time_plot, error, sats, accel, gyro) = plt.subplots(5,1, sharex=True)
    fig1. suptitle('time / data checks')
    time_plot.plot(gpsGenPosix, gpsVnGenPosix, label='gps time vn (s)')
    time_plot.plot(gpsGenPosix, gpsGenPosixCorrected, label='corrected gps time (s)')
    time_plot.legend()

    error.plot(gpsGenPosix, gpsErrorPosix, label='vn time - novatel time (s)')
    error.legend()

    sats.plot(gpsGenPosix, vnSats, label='vn sat count')
    sats.legend()

    accel.plot(gpsGenPosix, vnAccelX, label='accel x (m/s/s)')
    accel.plot(gpsGenPosix, vnAccelY, label='accel y (m/s/s)')
    accel.plot(gpsGenPosix, vnAccelZ, label='accel z (m/s/s)')
    accel.legend()

    gyro.plot(gpsGenPosix, vnGyroX, label='gyro x (deg/sec)')
    gyro.plot(gpsGenPosix, vnGyroY, label='gyro y (deg/sec)')
    gyro.plot(gpsGenPosix, vnGyroZ, label='gyro z (deg/sec)')
    gyro.legend()

    plt.show()

if __name__=='__main__':
    main()

