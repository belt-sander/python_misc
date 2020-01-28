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
    gpsTimeTest = inputData[:,87] 
    avgSpd = inputData[:,1] # km/h
    wheelSpeedFL = inputData[:,90] # m/s
    wheelSpeedFR = inputData[:,91] # mph ughhh...
    wheelSpeedRL = inputData[:,81] # m/s
    wheelSpeedRR = inputData[:,92] # mph ughhh...
    steerWheelAngleRate = inputData[:,94]
    steerWheelAngle = inputData[:,93]
    brakePres = inputData[:,107]
    vnYaw = inputData[:,57]
    vnPitch = inputData[:,60]
    vnRoll = inputData[:,61]
    vnVelNorth = inputData[:,56]
    vnVelEast = inputData[:,55]
    vnVelDown = inputData[:,62]
    vnAccelX = inputData[:,63]
    vnAccelY = inputData[:,64]
    vnAccelZ = inputData[:,65]
    vnGyroX = inputData[:,66]
    vnGyroY = inputData[:,67]
    vnGyroZ = inputData[:,68]
    rgLat = inputData[:,100]
    rgLong = inputData[:,101]
    novYaw = inputData[:,19]
    novEastVel = inputData[:,33]
    novNorthVel = inputData[:,32]
    novUpVel = inputData[:,34]

    ### rotation test ###
    vnRelVelX = np.zeros((len(inputData),1))
    vnRelVelY = np.zeros((len(inputData),1))
    novRelVelY = np.zeros((len(inputData),1))
    novRelVelX = np.zeros((len(inputData),1))    
    vnSpd = np.zeros((len(inputData),1))
    novSpd = np.zeros((len(inputData),1))

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
        gpsSec = row[87]
        gpsGenPosix[i,:] = gpsSec + timeOffset
        # print("new posix time:")
        # print(gpsGenPosix)
        forVnYaw =  row[57]*(np.pi/180)
        forNovYaw = (row[19]-2.5)*(np.pi/180) ### added value to try and calibrate out lateral velocity error
        forVnVelNorth = row[56]
        forVnVelEast = row[55]
        forVnVelDown = row[62]
        forNovEastVel = row[112]
        forNovNorthVel = row[114]
        forNovUpVel = row[116]

        vnRelVelX[i,:] = (np.sin(forVnYaw)*forVnVelEast + np.cos(forVnYaw)*forVnVelNorth)
        vnRelVelY[i,:] = (np.cos(forVnYaw)*forVnVelEast + (-np.sin(forVnYaw)*forVnVelNorth))
        novRelVelY[i,:] = (np.sin(forNovYaw)*forNovEastVel + np.cos(forNovYaw)*forNovNorthVel)
        novRelVelX[i,:] = (np.cos(forNovYaw)*forNovEastVel + (-np.sin(forNovYaw)*forNovNorthVel))
        vnSpd[i,:] = np.sqrt((forVnVelNorth**2)+(forVnVelEast**2)+(forVnVelDown**2))
        novSpd[i,:] = np.sqrt((forNovNorthVel**2)+(forNovEastVel**2)+(forNovUpVel**2))
        forWheelSpeedFR[i,:] = row[91] / 2.237 # mph to m/s
        forWheelSpeedRR[i,:] = row[92] / 2.237 # mph to m/s        

    dataOutput = np.column_stack((  gpsGenPosix,wheelSpeedFL,forWheelSpeedFR,
                                    wheelSpeedRL,forWheelSpeedRR,steerWheelAngle,brakePres,
                                    vnYaw,vnPitch,vnRoll,vnAccelX,
                                    vnAccelY,vnAccelZ,vnGyroX,vnGyroY,
                                    vnGyroZ,rgLat,rgLong, vnVelNorth, 
                                    vnVelEast, vnVelDown, novEastVel, novNorthVel, 
                                    novUpVel    ))
    np.savetxt(args.output, dataOutput, fmt='%.8f', delimiter=' ', header="# gpsGenPosix(s),wheelSpeedFL(m/s),wheelSpeedFR(m/s),wheelSpeedRL(m/s),wheelSpeedRR(m/s),steerWheelAngle(deg),BrakePres(unitless),vnYaw(deg),vnPitch(deg),vnRoll(deg),vnAccelX(m/s/s),vnAccelY(m/s/s),vnAccelZ(m/s/s),vnGyroX(deg/s),vnGyroY(deg/s),vnGyroZ(deg/s), rg lat(dd), rg long(dd), vnVelX(mph), vnVelY(mph), vnVelZ(mph), novEastVel(mph), novNorthVel(mph), novUpVel(mph)", comments='')
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
    plt.plot(gpsGenPosix, wheelSpeedFL, color='green', label='fl wheel (CAN) m/s')
    plt.plot(gpsGenPosix, forWheelSpeedFR, color='blue', label='fr wheel (CAN) m/s')

    ### experiments
    plt.plot(gpsGenPosix, vnRelVelY, color='red', label='vn rel Y (lateral vel)')
    plt.plot(gpsGenPosix, vnRelVelX, color='orange', label='vn rel X (forward vel)')
    plt.plot(gpsGenPosix, novRelVelY, label='novatel rel Y (forward vel)')
    plt.plot(gpsGenPosix, novRelVelX, label='novatel rel X (lateral vel)')
    plt.plot(gpsGenPosix, vnSpd, color='black', label='vn speed mag')
    plt.plot(gpsGenPosix, novSpd, label='novatel speed mag')
    plt.plot(gpsGenPosix, zero)
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
    plt.plot(gpsGenPosix, vnVelNorth, color='red', label='north vel (m/s)')
    plt.plot(gpsGenPosix, vnVelEast, color='blue', label='east vel (m/s)')
    plt.plot(gpsGenPosix, vnVelDown, color='green', label='down vel (m/s)')
    plt.plot(gpsGenPosix, novEastVel, label='novatel east vel (m/s)')
    plt.plot(gpsGenPosix, novNorthVel, label='novatel north vel (m/s)')
    plt.ylabel('vel (m/s)')
    plt.legend()

    ### orientation
    plt.subplot(312)
    plt.plot(gpsGenPosix, vnYaw, label='vn yaw (deg)')
    plt.plot(gpsGenPosix, vnPitch, label='vn pitch (deg)')
    plt.plot(gpsGenPosix, vnRoll, label='vn roll (deg)')
    plt.legend()

    ### gyro z 
    plt.subplot(313)
    plt.plot(gpsGenPosix, vnGyroZ, label='gyro Z (deg/sec)')
    plt.plot(gpsGenPosix, vnGyroY, label='gyro Y (deg/sec)')
    plt.plot(gpsGenPosix, vnGyroX, label='gyro X (deg/sec)')     
    plt.xlabel('utc time (s)')
    plt.legend()

    plt.show()

if __name__=='__main__':
    main()

