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

    inputData = np.genfromtxt(args.input_csv, skip_header=100, delimiter=',')
    print("data has been imported")
    print("")

    gpsGenPosix = np.zeros((len(inputData),1))
    sysTime = inputData[:,0]
    gpsTimeTest = inputData[:,63]
    engineSpd = inputData[:,4]
    gpsLat = inputData[:,74]
    gpsLong = inputData[:,75]
    avgSpd = inputData[:,2]
    brakeState = inputData[:,7]
    wheelSpeedFL = inputData[:,46]
    wheelSpeedFR = inputData[:,47]
    wheelSpeedRL = inputData[:,48]
    wheelSpeedRR = inputData[:,49]
    steerWheelAngle = inputData[:,54]
    wheelMoveState = inputData[:,55]
    brakePresF = inputData[:,56]
    brakePresR = inputData[:,57]
    avgBrakePres = brakePresR+brakePresF
    vnYaw = inputData[:,8]
    vnPitch = inputData[:,9]
    vnRoll = inputData[:,10]
    vnVelX = inputData[:,11]
    vnVelY = inputData[:,12]
    vnVelZ = inputData[:,13]
    vnAccelX = inputData[:,16]
    vnAccelY = inputData[:,17]
    vnAccelZ = inputData[:,18]
    vnGyroX = inputData[:,19]
    vnGyroY = inputData[:,20]
    vnGyroZ = inputData[:,21]
    rgLat = inputData[:,14]
    rgLong = inputData[:,15]
    wheelOdoRL = inputData[:,80]
    wheelOdoRR = inputData[:,81]
    novYaw = inputData[:,100]
    novEastVel = inputData[:,102]
    novNorthVel = inputData[:,104]
    novUpVel = inputData[:,106]
    # speedOBDResponse = inputData[:,49]

    ### rotation test ###
    vnRelVelX = np.zeros((len(inputData),1))
    vnRelVelY = np.zeros((len(inputData),1))
    novRelVelY = np.zeros((len(inputData),1))
    novRelVelX = np.zeros((len(inputData),1))    
    vnSpd = np.zeros((len(inputData),1))
    vnPitch = inputData[:, 9]#*(180/np.pi)
    vnRoll = inputData[:, 10]#*(180/np.pi)
    vnYaw = inputData[:, 8]#*(180/np.pi)    

    print("iterating through data...")
    print("")
    for i, row in enumerate(inputData):
        gpsSec = row[63]
        gpsGenPosix[i,: ] = gpsSec + timeOffset
        # print("new posix time:")
        # print(gpsGenPosix)
        forVnYaw =  row[8]*(np.pi/180)
        forNovYaw = row[100]*(np.pi/180)
        forVnVelX = row[11]
        forVnVelY = row[12]
        forVnVelZ = row[13]
        forNovEastVel = row[102]
        forNovNorthVel = row[104]
        forNovUpVel = row[106]
        vnRelVelX[i,:] = (np.sin(forVnYaw)*forVnVelY + np.cos(forVnYaw)*forVnVelX)
        vnRelVelY[i,:] = (np.cos(forVnYaw)*forVnVelY + (-np.sin(forVnYaw)*forVnVelX))
        novRelVelY[i,:] = (np.sin(forNovYaw)*forNovEastVel + np.cos(forNovYaw)*forNovNorthVel)
        novRelVelX[i,:] = (np.cos(forNovYaw)*forNovEastVel + (-np.sin(forNovYaw)*forNovNorthVel))
        vnSpd[i,:] = np.sqrt((forVnVelX**2)+(forVnVelY**2)+(forVnVelZ**2))        

    dataOutput = np.column_stack((gpsGenPosix,avgSpd,brakeState,wheelOdoRR,wheelOdoRL,wheelSpeedFL,wheelSpeedFR,wheelSpeedRL,wheelSpeedRR,steerWheelAngle,wheelMoveState,avgBrakePres,vnYaw,vnPitch,vnRoll,vnAccelX,vnAccelY,vnAccelZ,vnGyroX,vnGyroY,vnGyroZ,gpsLat,gpsLong,rgLat,rgLong, vnVelX, vnVelY, vnVelZ))
    np.savetxt(args.output, dataOutput, fmt='%.8f', delimiter=' ', header="# gpsGenPosix(s),avgSpd(mph),brakeState(unitless),wheelodometeryRR(mph),wheelodometeryRL(mph),wheelSpeedFL(mph),wheelSpeedFR(mph),wheelSpeedRL(mph),wheelSpeedRR(mph),steerWheelAngle(deg),wheelMoveState(unitless),avgBrakePres(unitless),vnYaw(deg),vnPitch(deg),vnRoll(deg),vnAccelX(m/s/s),vnAccelY(m/s/s),vnAccelZ(m/s/s),vnGyroX(deg/s),vnGyroY(deg/s),vnGyroZ(deg/s),gps lat(dd), gps long(dd), rg lat(dd), rg long(dd), vnVelX(m/s), vnVelY(m/s), vnVelZ(m/s)", comments='')
    print("data has been exported")
    print("")


    plt.title("relevant vehicle data")
    plt.figure(1)

    ### can wheel speeds and move states
    # plt.plot(gpsGenPosix, wheelSpeedRR, color='gold', label='rr wheel')
    # plt.plot(gpsGenPosix, wheelSpeedRL, color='red', label='rl wheel')
    # plt.plot(gpsGenPosix, wheelMoveState, color='red', label='wheels moving state')

    plt.plot(gpsGenPosix, wheelSpeedFL, '-o', color='green', label='fl wheel (CAN)')
    plt.plot(gpsGenPosix, wheelSpeedFR, '-o', color='blue', label='fr wheel (CAN)')
    plt.plot(gpsGenPosix, wheelOdoRR, '-o', color='magenta', label='rr direct (Encoder)')
    plt.plot(gpsGenPosix, wheelOdoRL, '-o', color='brown', label='rl direct (Encoder)')

    ### experiments
    plt.plot(gpsGenPosix, vnRelVelY, color='red', label='vn rel Y (lateral vel)')
    plt.plot(gpsGenPosix, vnRelVelX, color='orange', label='vn rel X (forward vel)')
    plt.plot(gpsGenPosix, novRelVelY, label='novatel rel Y (forward vel)')
    plt.plot(gpsGenPosix, novRelVelX, label='novatel rel X (lateral vel)')
    plt.plot(gpsGenPosix, vnSpd, color='black', label='vn spd calcd')
    # plt.plot(gpsGenPosix, speedOBDResponse, color='cyan', label='obd speed response (OBD)')
    
    plt.ylabel('wheel speed (mph)')
    plt.legend()
    plt.xlabel('utc time (s)')

    plt.figure(2)
    plt.title("vectornav / racegrade compare")
    plt.plot(rgLat,rgLong,color='red')
    plt.plot(gpsLat,gpsLong,color='green')
    plt.ylabel("longitude (dd)")
    plt.xlabel("latitude (dd)")

    plt.figure(3)
    plt.subplot(311)
    plt.title("global velocity")
    plt.plot(gpsGenPosix, vnVelX, color='red', label='north vel (m/s)')
    plt.plot(gpsGenPosix, vnVelY, color='blue', label='east vel (m/s)')
    plt.plot(gpsGenPosix, vnVelZ, color='green', label='down vel (m/s)')
    plt.plot(gpsGenPosix, novEastVel, label='novatel east vel (m/s)')
    plt.plot(gpsGenPosix, novNorthVel, label='novatel north vel (m/s)')
    plt.ylabel('vel (m/s)')
    plt.legend()

    plt.subplot(312)
    plt.plot(gpsGenPosix, vnYaw, label='yaw (deg)')
    plt.plot(gpsGenPosix, vnPitch, label='pitch (deg)')
    plt.plot(gpsGenPosix, vnRoll, label='roll (deg)')
    plt.legend()

    plt.subplot(313)
    plt.plot(gpsGenPosix, vnGyroZ, label='gyro Z (deg/sec)')
    # plt.plot(gpsGenPosix, vnGyroY, '-o', label='gyro Y (deg/sec)')
    # plt.plot(gpsGenPosix, vnGyroX, '-o', label='gyro X (deg/sec)')     
    plt.xlabel('utc time (s)')
    plt.legend()

    plt.show()

if __name__=='__main__':
    main()

