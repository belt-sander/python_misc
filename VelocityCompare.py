#!/usr/bin/env python

from __future__ import print_function
import matplotlib.pyplot as plt
import numpy as np
import argparse
from pyproj import Proj
from scipy.signal import savgol_filter
import time, datetime

def parse_args():
    DEFAULT_OUTPUT_DIR = ''
    arg_parser = argparse.ArgumentParser(description='Generate trajectory comparison plots')
    arg_parser.add_argument('-g',
                            '--truthTrajectory',
                            required=True,
                            help='Txt file containing the reference trajectory')
    arg_parser.add_argument('-c',
                            '--compareTrajectory',
                            required=True,
                            help='Txt file containing the query trajectory')
    return arg_parser.parse_args()

def main():
    args = parse_args()
    truth = np.genfromtxt(args.truthTrajectory, delimiter=',', skip_header=2, skip_footer=0, dtype=None)
    compare = np.genfromtxt(args.compareTrajectory, delimiter=' ', skip_header=0)
    print()
    print("data imported.", '\n')
        
    myProjTruth = Proj("+proj=utm +zone=10S, +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs")
    
    truth_utm_n = np.zeros((len(truth),1))
    truth_utm_e = np.zeros((len(truth),1))
    truth_vel_fwd = np.zeros((len(truth),1))
    truth_vel_lat = np.zeros((len(truth),1))
    # gps_time_truth = np.zeros((len(truth),1))
    gps_time_truth = []

    gps_time_compare = np.zeros((len(compare),1))
    compare_vel_fwd = np.zeros((len(compare),1))

    for i, row in enumerate(truth):
        _year = 2020
        _month = 5
        _day = 13
        _time_in = row[1]
        _h = int(_time_in[:2])
        _m = int(_time_in[:5][3:])
        _s = int(_time_in[:8][6:])
        _ms = int(_time_in[:12][9:])
        _posix_s = time.mktime(datetime.datetime(_year,_month,_day,_h, _m, _s, _ms).timetuple())
        _posix_ms = _posix_s + (_ms * 1e-3)
        
        # if _posix_ms >= (1.58942e9 + 3400) and _posix_ms <= (1.58942e9 + 4100):

        # gps_time_truth[i,: ] = _posix_ms
        gps_time_truth.append(_posix_ms)
        _truth_lat = row[3]
        _truth_lon = row[4]
        _truth_fwd_vel = row[14]
        _truth_lat_vel = row[15]
        
        utmNTruth,utmETruth = myProjTruth(_truth_lon,_truth_lat)
        truth_utm_n[i,: ] = utmNTruth
        truth_utm_e[i,: ] = utmETruth 
        truth_vel_fwd[i,: ] = _truth_fwd_vel
        truth_vel_lat[i,: ] = _truth_lat_vel

    for i, row in enumerate(compare):
        _gps_time_compare = row[0]
        _compare_fwd_vel = row[4]

        gps_time_compare[i,: ] = _gps_time_compare + 10800 + 3600
        compare_vel_fwd[i,: ] = _compare_fwd_vel * 1.02325    


    truth_utm_nArr = np.resize(truth_utm_n,np.size(truth_utm_n))
    truth_utm_eArr = np.resize(truth_utm_e,np.size(truth_utm_e))

    ### compare velocity in to truth time frame ###
    resize_gps_time_compare = np.resize(gps_time_compare, np.size(gps_time_compare))
    resize_compare_vel_fwd = np.resize(compare_vel_fwd, np.size(compare_vel_fwd))
    interp_compare_fwd_vel = np.interp(gps_time_truth, resize_gps_time_compare, resize_compare_vel_fwd)

    ### truth velocity in to compare time frame ###
    resize_gps_time_truth = np.resize(gps_time_truth, np.size(gps_time_truth))
    resize_truth_vel_fwd = np.resize(truth_vel_fwd, np.size(truth_vel_fwd))
    interp_compare_fwd_vel_truth = np.interp(gps_time_compare, resize_gps_time_truth, resize_truth_vel_fwd)

    print('mean vel truth (km/h): ', np.mean(interp_compare_fwd_vel_truth))
    print('mean vel CAN (km/h): ', np.mean(compare_vel_fwd * 3.6))
    print('std vel truth (km/h): ', np.std(interp_compare_fwd_vel_truth))
    print('std vel CAN (km/h): ', np.std(compare_vel_fwd * 3.6), '\n')

    ve_0 = []
    ve_1 = []
    ve_2 = []
    ve_3 = []
    ve_4 = []
    ve_5 = []
    ve_6 = []
    ve_7 = []
    ve_8 = []

    for i, row in enumerate(interp_compare_fwd_vel_truth):
        if (interp_compare_fwd_vel_truth[i] / 3.6) < 2.5:
            ve_0.append(interp_compare_fwd_vel_truth[i]/3.6 - compare_vel_fwd[i]) 
        elif (interp_compare_fwd_vel_truth[i] / 3.6) >= 2.5 and (interp_compare_fwd_vel_truth[i] / 3.6) < 5:
            ve_1.append(interp_compare_fwd_vel_truth[i]/3.6 - compare_vel_fwd[i])
        elif (interp_compare_fwd_vel_truth[i] / 3.6) >= 5 and (interp_compare_fwd_vel_truth[i] / 3.6) < 7.5:
            ve_2.append(interp_compare_fwd_vel_truth[i]/3.6 - compare_vel_fwd[i])
        elif (interp_compare_fwd_vel_truth[i] / 3.6) >= 7.5 and (interp_compare_fwd_vel_truth[i] / 3.6) < 10:
            ve_3.append(interp_compare_fwd_vel_truth[i]/3.6 - compare_vel_fwd[i])
        elif (interp_compare_fwd_vel_truth[i] / 3.6) >= 10 and (interp_compare_fwd_vel_truth[i] / 3.6) < 12.5:
            ve_4.append(interp_compare_fwd_vel_truth[i]/3.6 - compare_vel_fwd[i])
        elif (interp_compare_fwd_vel_truth[i] / 3.6) >= 12.5 and (interp_compare_fwd_vel_truth[i] / 3.6) < 15:
            ve_5.append(interp_compare_fwd_vel_truth[i]/3.6 - compare_vel_fwd[i])
        elif (interp_compare_fwd_vel_truth[i] / 3.6) >= 15 and (interp_compare_fwd_vel_truth[i] / 3.6) < 17.5:
            ve_6.append(interp_compare_fwd_vel_truth[i]/3.6 - compare_vel_fwd[i])
        elif (interp_compare_fwd_vel_truth[i] / 3.6) >= 17.5 and (interp_compare_fwd_vel_truth[i] / 3.6) < 20:
            ve_7.append(interp_compare_fwd_vel_truth[i]/3.6 - compare_vel_fwd[i])
        elif (interp_compare_fwd_vel_truth[i] / 3.6) >= 20 and (interp_compare_fwd_vel_truth[i] / 3.6) < 22.5:
            ve_8.append(interp_compare_fwd_vel_truth[i]/3.6 - compare_vel_fwd[i])
        elif (interp_compare_fwd_vel_truth[i] / 3.6) >= 22.5 and (interp_compare_fwd_vel_truth[i] / 3.6) < 25:
            ve_9 = interp_compare_fwd_vel_truth[i]/3.6 - compare_vel_fwd[i]
        # elif (interp_compare_fwd_vel_truth[i] / 3.6) >= 25 and (interp_compare_fwd_vel_truth[i] / 3.6) < 27.5:
        #     ve_10 = interp_compare_fwd_vel_truth[i]/3.6 - compare_vel_fwd[i]

    print('mean error from 0 - 2.5   (m/s): ', np.mean(ve_0), ' size: ', np.size(ve_0))
    print('mean error from 2.5 - 5   (m/s): ', np.mean(ve_1), ' size: ', np.size(ve_1))
    print('mean error from 5 - 7.5   (m/s): ', np.mean(ve_2), ' size: ', np.size(ve_2))
    print('mean error from 7.5 - 10  (m/s): ', np.mean(ve_3), ' size: ', np.size(ve_3))
    print('mean error from 10 - 12.5 (m/s): ', np.mean(ve_4), ' size: ', np.size(ve_4))
    print('mean error from 12.5 - 15 (m/s): ', np.mean(ve_5), ' size: ', np.size(ve_5))
    print('mean error from 15 - 17.5 (m/s): ', np.mean(ve_6), ' size: ', np.size(ve_6))
    print('mean error from 17.5 - 20 (m/s): ', np.mean(ve_7), ' size: ', np.size(ve_7))
    print('mean error from 20 - 22.5 (m/s): ', np.mean(ve_8), ' size: ', np.size(ve_8))
    print('mean error from 22.5 - 25 (m/s): ', np.mean(ve_9), ' size: ', np.size(ve_9))

    breakpoints = ['(0) 0 - 2.5 (m/s)', '(1) 2.5 - 5 (m/s)', '(2) 5 - 7.5 (m/s)', '(3) 7.5 - 10 (m/s)', '(4) 10 - 12.5 (m/s)', '(5) 12.5 - 15 (m/s)', '(6) 15 - 17.5 (m/s)', '(7) 17.5 - 20 (m/s)', '(8) 20 - 22.5 (m/s)', '(9) 22.5 - 25 (m/s)']
    results = [np.mean(ve_0), np.mean(ve_1), np.mean(ve_2), np.mean(ve_3), np.mean(ve_4), np.mean(ve_5), np.mean(ve_6), np.mean(ve_7), np.mean(ve_8), np.mean(ve_9)]

    velocity_error = (interp_compare_fwd_vel_truth/3.6) - compare_vel_fwd

    plt.figure(1)
    plt.plot(gps_time_compare, compare_vel_fwd, label='forward vel (m/s)')
    plt.plot(gps_time_compare, interp_compare_fwd_vel_truth / 3.6, label='truth vel (m/s)')
    plt.legend()

    plt.figure(2)
    plt.plot(truth_utm_eArr, truth_utm_nArr, color='green', label='novatel (truth)')
    plt.ylabel("utm northing (m)")
    plt.xlabel("utm easting (m)")
    plt.legend()

    fig, (vx) = plt.subplots(1,1, sharex=True, sharey=True)
    fig.suptitle('forward velocity error')
    vx.hist(velocity_error, bins=100, label='can velocity', density=True, facecolor='g', alpha=0.75)
    vx.set_xlabel('error (m/s)')
    vx.set_ylabel('speed (m/s)')
    vx.legend()

    plt.figure(4)
    plt.bar(breakpoints, results)
    plt.ylabel('mean error (m/s)')

    plt.show()

if __name__=='__main__':
    main()