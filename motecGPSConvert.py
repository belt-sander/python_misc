#simple tool written to convert stupid motec gps time to posix time

#!/usr/bin/python

import time
import csv
import argparse
import datetime
import numpy as np

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
                            help='Date (day, month, year) that data was recorded')
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

    inputData = np.genfromtxt(args.input_csv, skip_header=16, delimiter=',')
    print("data has been imported")
    print("")

    gpsGenPosix = np.zeros((len(inputData),1))
    avgSpd = inputData[:,1]
    brakeState = inputData[:,2]
    wheelSpeedFL = inputData[:,18]
    wheelSpeedFR = inputData[:,19]
    wheelSpeedRL = inputData[:,20]
    wheelSpeedRR = inputData[:,21]
    steerWheelAngle = inputData[:,22]
    wheelMoveState = inputData[:,23]
    brakePresF = inputData[:,24]
    brakePresR = inputData[:,25]
    avgBrakePres = brakePresR+brakePresF
    
    # gpsSec = inputData[:,26]
    # print("utc seconds of day:")
    # print(gpsSec)
    # print("")

    print("iterating through data...")
    print("")
    for i, row in enumerate(inputData):
        gpsSec = row[26]
        gpsGenPosix[i,: ] = gpsSec + timeOffset
        # print("new posix time:")
        # print(gpsGenPosix)

    dataOutput = np.column_stack((gpsGenPosix,avgSpd,brakeState,wheelSpeedFL,wheelSpeedFR,wheelSpeedRL,wheelSpeedRR,steerWheelAngle,wheelMoveState,avgBrakePres))
    np.savetxt(args.output, dataOutput, fmt='%.8f', delimiter=' ', header="# gpsGenPosix,avgSpd,brakeState,wheelSpeedFL,wheelSpeedFR,wheelSpeedRL,wheelSpeedRR,steerWheelAngle,wheelMoveState,avgBrakePres", comments='')
    print("data has been exported")
    print("")


if __name__=='__main__':
    main()

