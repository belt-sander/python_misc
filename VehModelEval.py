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
    arg_parser.add_argument('-v', '--vehicleData', required=True, help='output from MoTeCGPSTimeConvert.py')
    arg_parser.add_argument('-n', '--novatelTrajectory', required=True, help='novatel post processed data (session.txt)')
    return arg_parser.parse_args()

def tuning():
    # tuning parameters for 2017 Honda Civic
    steering_ratio = 10.98 # ratio
    steering_offset = 1.1 # degrees
    wheel_base = 2.7 # meters 106.3 inches
    track_width = 1.5621 # meters 61.5 inches rear  
    rack_scalar = -1.0 # ratio
    speed_scalar = 0.98 # ratio

    return steering_ratio, steering_offset, wheel_base, track_width, rack_scalar, speed_scalar

def novatel_data():
    args = parse_args() 
    truth = np.genfromtxt(args.novatelTrajectory, skip_header=50, skip_footer=50, delimiter='')

    # parse novatel data
    gps_time_truth = np.zeros((len(truth),1))
    gps_northing_truth = np.zeros((len(truth),1))
    gps_easting_truth = np.zeros((len(truth),1))
    gps_heading_truth = np.zeros((len(truth),1))
    calc_heading_offset = np.zeros((len(truth),1))
    gps_body_vel_x = np.zeros((len(truth),1))
    gps_body_vel_y = np.zeros((len(truth),1))
    gps_body_vel_z = np.zeros((len(truth),1))

    # LLA to UTM conversion
    myProjTruth = Proj("+proj=utm +zone=10S, +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs")
    print('')
    print('converting truth lla to UTM...')
    print('')

    for aa, row in enumerate(truth):
        _gps_time_truth = row[0]
        _gps_lat_truth = row[1]
        _gps_long_truth = row[2]
        _gps_heading_truth = row[13]-2.84  ### IMPORTANT: this is to offset the imperfect installation of the IMU in the Honda
        _gps_body_vel_x = row[19]
        _gps_body_vel_y = row[20]
        _gps_body_vel_z = row[21]

        # convert from -180 to 180 to 0 to 360
        if _gps_heading_truth < 0:
            calc_heading_offset[aa,: ] = (_gps_heading_truth + 360) * np.pi/180
        else:
            calc_heading_offset[aa,: ] = (_gps_heading_truth) * np.pi/180

        _utm_e_truth, _utm_n_truth = myProjTruth(_gps_long_truth,_gps_lat_truth)
        gps_time_truth[aa,: ] = _gps_time_truth
        gps_northing_truth[aa,: ] = _utm_n_truth
        gps_easting_truth[aa,: ] = _utm_e_truth 
        gps_heading_truth[aa,: ] = _gps_heading_truth
        gps_body_vel_x[aa,:] = _gps_body_vel_x
        gps_body_vel_y[aa,:] = _gps_body_vel_y
        gps_body_vel_z[aa,:] = _gps_body_vel_z

    return gps_time_truth, gps_northing_truth, gps_easting_truth, gps_heading_truth, gps_body_vel_x, gps_body_vel_y, gps_body_vel_z


def vehicle_data():
    args = parse_args() 
    vehicle = np.genfromtxt(args.vehicleData, skip_header=1, skip_footer=1, delimiter=' ')

    # parse vehicle data
    vn_accel_x = np.zeros((len(vehicle),1))
    vn_accel_y = np.zeros((len(vehicle),1))
    vn_accel_z = np.zeros((len(vehicle),1))
    avg_rear_axle_speed = np.zeros((len(vehicle),1))
    gps_time_vehicle = np.zeros((len(vehicle),1))
    steering_wheel_angle = np.zeros((len(vehicle),1))
    wheel_speed_fl = np.zeros((len(vehicle),1))
    wheel_speed_fr = np.zeros((len(vehicle),1))
    wheel_speed_rl = np.zeros((len(vehicle),1))
    wheel_speed_rr = np.zeros((len(vehicle),1))

    for i, row in enumerate(vehicle):
        _gps_time_vehicle = row[0]
        _steer_wheel_angle = row[5]
        _vn_accel_x = row[10]
        _vn_accel_y = row[11]
        _vn_accel_z = row[12]
        _wheel_speed_fl = row[1]
        _wheel_speed_fr = row[2]
        _wheel_speed_rl = row[3]
        _wheel_speed_rr = row[4]

        vn_accel_x[i,:] = _vn_accel_x
        vn_accel_y[i,:] = _vn_accel_y
        vn_accel_z[i,:] = _vn_accel_z
        avg_rear_axle_speed[i,:] = (_wheel_speed_rl+_wheel_speed_rr)/2 # speed at center of rear axle
        gps_time_vehicle[i,:] = _gps_time_vehicle
        steering_wheel_angle[i,:] = _steer_wheel_angle
        wheel_speed_fl[i,:] = _wheel_speed_fl
        wheel_speed_fr[i,:] = _wheel_speed_fr
        wheel_speed_rl[i,:] = _wheel_speed_rl
        wheel_speed_rr[i,:] = _wheel_speed_rr

    return gps_time_vehicle, wheel_speed_fl, wheel_speed_fr, wheel_speed_rl, wheel_speed_rr, steering_wheel_angle, vn_accel_x


def main():
    tuning_values = tuning()
    vehicle_state = vehicle_data()
    novatel_state = novatel_data()

    evaluation_gps_time = np.resize(vehicle_state[0], np.size(vehicle_state[0]))
    steering_wheel_angle = np.resize(vehicle_state[5], np.size(vehicle_state[5])) 
    steering_rack_angle = np.resize((vehicle_state[5] + tuning_values[1] * tuning_values[4]) / tuning_values[0], np.size(vehicle_state[5]))        # (steering wheel angle + steering wheel offset) / steering ratio
    avg_rear_axle_speed = np.resize(((vehicle_state[3] + vehicle_state[4]) * tuning_values[5] / 2), np.size(vehicle_state[0]))  # (rear left wheel speed + rear right wheel speed * speed scalar) / 2
    wheel_speed_fl = np.resize(vehicle_state[1], np.size(vehicle_state[1]))
    wheel_speed_fr = np.resize(vehicle_state[2], np.size(vehicle_state[2]))
    wheel_speed_rl = np.resize(vehicle_state[3], np.size(vehicle_state[3]))
    wheel_speed_rr = np.resize(vehicle_state[4], np.size(vehicle_state[4]))

    # used to evaluate data latency
    calc_forward_accel = np.gradient(avg_rear_axle_speed) * 100.0 # multiplier due to sample rate at 100hz
    filter_calc_forward_accel = savgol_filter(calc_forward_accel, 49, 1) 
    forward_accel = np.resize(vehicle_state[6], np.size(vehicle_state[6]))

    # putting vehicle state data in the novatel state data time reference
    interp_steering_wheel_angle = np.interp(novatel_state[0], evaluation_gps_time, steering_wheel_angle)
    interp_avg_rear_axle_speed = np.interp(novatel_state[0], evaluation_gps_time, avg_rear_axle_speed)
    interp_forward_accel = np.interp(novatel_state[0], evaluation_gps_time, forward_accel)
    interp_calc_forward_accel = np.interp(novatel_state[0], evaluation_gps_time, calc_forward_accel)
    interp_filter_calc_forward_accel = np.interp(novatel_state[0], evaluation_gps_time, filter_calc_forward_accel)
    interp_wheel_speed_fl = np.interp(novatel_state[0], evaluation_gps_time, wheel_speed_fl)
    interp_wheel_speed_fr = np.interp(novatel_state[0], evaluation_gps_time, wheel_speed_fr)
    interp_wheel_speed_rl = np.interp(novatel_state[0], evaluation_gps_time, wheel_speed_rl)
    interp_wheel_speed_rr = np.interp(novatel_state[0], evaluation_gps_time, wheel_speed_rr)
    interp_steering_rack_angle = np.interp(novatel_state[0], evaluation_gps_time, steering_rack_angle) 

    # relative novatel position
    gps_northing = novatel_state[1]
    gps_easting = novatel_state[2]
    relative_gps_northing = gps_northing - gps_northing[0]
    relative_gps_easting = gps_easting - gps_easting[0]

    # relative novatel heading
    gps_heading = novatel_state[3] * np.pi/180
    relative_gps_heading = gps_heading - gps_heading[0]


    ### theta calculation ###
    theta = np.zeros((len(novatel_state[0]),1))
    theta_global = np.zeros((len(novatel_state[0]),1))
    theta_error_uncorrected = np.zeros((len(novatel_state[0]),1))
    theta_error_corrected = np.zeros((len(novatel_state[0]),1))
    relative_gps_heading_corrected = np.zeros((len(novatel_state[0]),1))
    zero = np.zeros((len(novatel_state[0]),1))

    combined_data = np.column_stack((novatel_state[0], interp_avg_rear_axle_speed, 
                                    interp_steering_rack_angle, relative_gps_northing, 
                                    relative_gps_easting, relative_gps_heading))
    for i, row in enumerate(combined_data):
        _time = row[0]
        _speed = row[1]
        _rack_angle = row[2]
        _rel_gps_northing = row[3]
        _rel_gps_easting = row[4]
        _rel_gps_heading = row[5]
        _wheel_base = tuning_values[2]

        ### theta ###
        dt = (i * 0.01)
        theta[i,:] = (_speed/_wheel_base) * (np.tan(_rack_angle * np.pi/180)) * 0.01 + theta[i-1]

        ### theta error ###
        theta_error_uncorrected[i,:] = _rel_gps_heading - theta[i]
        
        if theta_error_uncorrected[i] < -5:
            relative_gps_heading_corrected[i,:] = _rel_gps_heading + 360 * (np.pi/180)
        elif theta_error_uncorrected[i] > 5:
            relative_gps_heading_corrected[i,:] = _rel_gps_heading - 360 * (np.pi/180)
        else:
            relative_gps_heading_corrected[i,:] = _rel_gps_heading

        theta_error_corrected[i,:] = relative_gps_heading_corrected[i] - theta[i]

    ### theta global correction ###
    theta_global = theta + gps_heading[0]

    ### 2D pose prediction ###
    x_prediction = np.zeros((len(novatel_state[0]),1))
    y_prediction = np.zeros((len(novatel_state[0]),1))

    prediction_time = 0
    start_sample = 0
    for prediction_time in range(len(novatel_state[0])):
        prediction_time + 1
        x_prediction[prediction_time,:] = interp_avg_rear_axle_speed[prediction_time] * np.cos(theta_global[prediction_time]) * 0.01 + x_prediction[prediction_time - 1]
        y_prediction[prediction_time,:] = interp_avg_rear_axle_speed[prediction_time] * np.sin(theta_global[prediction_time]) * 0.01 + y_prediction[prediction_time - 1]

    ### distance traveled ###
    distance_traveled_novatel = np.cumsum(novatel_state[5]) * 0.01 # scalar for sample rate
    distance_traveled_can = np.cumsum(interp_avg_rear_axle_speed) * 0.01 # scalar for sample rate
    distance_traveled_error = ((distance_traveled_novatel - distance_traveled_can) / distance_traveled_novatel) * 100.0

    ### plots ### 
    fig1, (vel, accel, ws, steer, error, theta, dist_trav, dist_trav_error) = plt.subplots(8,1, sharex=True)
    fig1.suptitle('entire dataset')
    vel.plot(novatel_state[0], interp_avg_rear_axle_speed, label='rear axle speed CAN (m/s)')
    vel.plot(novatel_state[0], novatel_state[5], label='forward velocity novatel (m/s)')
    vel.legend()
    
    accel.plot(novatel_state[0], interp_forward_accel, label='vn accel forward (m/s/s)')
    accel.plot(novatel_state[0], interp_calc_forward_accel, label='calc accel forward (m/s/s)')
    accel.plot(novatel_state[0], interp_filter_calc_forward_accel, label='filtered calc accel forward (m/s/s)')
    accel.legend()

    ws.plot(novatel_state[0], interp_wheel_speed_fl, label='fl wheel speed (m/s)')
    ws.plot(novatel_state[0], interp_wheel_speed_fr, label='fr wheel speed (m/s)')
    ws.plot(novatel_state[0], interp_wheel_speed_rl, label='rl wheel speed (m/s)')
    ws.plot(novatel_state[0], interp_wheel_speed_rr, label='rr wheel speed (m/s)')
    ws.legend()

    steer.plot(novatel_state[0], interp_steering_wheel_angle, label='steering wheel angle (deg)')
    steer.plot(novatel_state[0], interp_steering_rack_angle, label='steering rack angle (deg)')
    steer.legend()

    error.plot(novatel_state[0], theta_error_corrected * 180/np.pi, label='corrected theta error (deg)')
    error.plot(novatel_state[0], zero)
    error.legend()

    theta.plot(novatel_state[0], theta_global, label='theta global (rad)')
    theta.plot(novatel_state[0], gps_heading, label='global heading (rad)')
    theta.legend()

    dist_trav.plot(novatel_state[0], distance_traveled_novatel, label='novatel distance travled (m)')
    dist_trav.plot(novatel_state[0], distance_traveled_can, label='can distance traveled (m)')
    dist_trav.legend()

    dist_trav_error.plot(novatel_state[0], distance_traveled_error, label='percent error (%)')
    dist_trav_error.plot(novatel_state[0], zero)
    dist_trav_error.set_ylim(-0.75,0.75)
    dist_trav_error.legend()

    plt.figure(2)
    plt.grid(b=True)
    plt.title('vehicle motion')
    plt.scatter(relative_gps_easting, relative_gps_northing, label='relative motion (utm)')
    plt.scatter(y_prediction, x_prediction, label='predicted path (m)')
    plt.legend()

    plt.show()


if __name__=='__main__':
    main()

