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
    compare = np.genfromtxt(args.compareTrajectory, delimiter=',', skip_header=2, skip_footer=0, dtype=None)
    print()
    print("data imported.", '\n')
        
    myProjTruth = Proj("+proj=utm +zone=10S, +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs")
    
    truth_utm_n = np.zeros((len(truth),1))
    truth_utm_e = np.zeros((len(truth),1))
    truth_vel_fwd = np.zeros((len(truth),1))
    truth_vel_lat = np.zeros((len(truth),1))
    gps_time_truth = []

    for i, row in enumerate(truth):
        _year = 2020
        _month = 5
        _day = 21
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

    gps_time_compare = []
    compare_vel_fwd  = []

    for i, row in enumerate(compare):
        _year = 2020
        _month = 5
        _day = 21
        _time_in = row[1]
        _h = int(_time_in[:2])
        _m = int(_time_in[:5][3:])
        _s = int(_time_in[:8][6:])
        _ms = int(_time_in[:12][9:])
        _posix_s = time.mktime(datetime.datetime(_year,_month,_day,_h, _m, _s, _ms).timetuple())
        _posix_ms = _posix_s + (_ms * 1e-3)
        _compare_fwd_vel = row[18]
        compare_vel_fwd.append(_compare_fwd_vel)    

        gps_time_compare.append(_posix_ms)
        # gps_time_compare[i,: ] = _gps_time_compare + 10800 + 3600
        
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
    print('mean vel ubx (km/h): ', np.mean(compare_vel_fwd))
    print('std vel truth (km/h): ', np.std(interp_compare_fwd_vel_truth))
    print('std vel ubx (km/h): ', np.std(compare_vel_fwd), '\n')

    plt.figure(1)
    plt.plot(gps_time_compare, compare_vel_fwd, label='ubx vel (km/h)')
    plt.plot(gps_time_compare, interp_compare_fwd_vel_truth, label='oxts vel (km/h)')
    plt.legend()

    plt.show()

if __name__=='__main__':
    main()