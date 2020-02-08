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

def tuning_honda():
    # tuning parameters for 2017 Honda Civic
    steering_ratio = 10.98
    steering_offset = 0.0
    steering_ratio_speed_scalar = 0.545 # 0.4625 is a good value for out and back dataset
    wheel_base = 2.7 # meters 106.3 inches
    track_width = 1.5621 # meters 61.5 inches rear  
    rack_scalar = 1.0 # ratio
    speed_scalar = 0.98  # ratio
    sample_rate = 100.0 # novatel sample rate hz
    lf_speed_correction = 1.0
    rf_speed_correction = 1.0
    lr_speed_correction = 0.99796
    rr_speed_correction = 1.0

    return steering_ratio, steering_offset, wheel_base, track_width, rack_scalar, speed_scalar, steering_ratio_speed_scalar, sample_rate, lf_speed_correction, rf_speed_correction, lr_speed_correction, rr_speed_correction

def tuning_ford():
    # tuning parameters for 2017 Ford F150 King Ranch
    steering_ratio = 16.75
    steering_offset = -1.0
    steering_ratio_speed_scalar = 0.2
    wheel_base = 3.683 # meters 145 inches
    track_width = 1.905 # meters 79.9 inches vehicle width. assume track width of 75 inches ??? 
    rack_scalar = -1.0 # ratio
    speed_scalar = 0.99  # ratio
    sample_rate = 200.0 # novatel sample rate hz
    lf_speed_correction = 1.0
    rf_speed_correction = 1.0
    lr_speed_correction = 1.0
    rr_speed_correction = 1.0

    return steering_ratio, steering_offset, wheel_base, track_width, rack_scalar, speed_scalar, steering_ratio_speed_scalar, sample_rate, lf_speed_correction, rf_speed_correction, lr_speed_correction, rr_speed_correction

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
    gps_gyro_z = np.zeros((len(truth),1))
    gps_calc_vel_forward = np.zeros((len(truth),1))
    gps_calc_vel_lateral = np.zeros((len(truth),1))

    # LLA to UTM conversion
    myProjTruth = Proj("+proj=utm +zone=10S, +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs")
    print('')
    print('converting truth lla to UTM...')

    for aa, row in enumerate(truth):
        _gps_time_truth = row[0]
        _gps_lat_truth = row[1]
        _gps_long_truth = row[2]
        _gps_gyro_z = row[9]
        _gps_heading_truth = row[13] - 2.90  ### IMPORTANT: this is to offset the imperfect installation of the IMU in the Honda
        _gps_body_vel_x = row[19]
        _gps_body_vel_y = row[20]
        _gps_body_vel_z = row[21]
        _gps_global_vel_north = row[16]
        _gps_global_vel_east = row[17]
        _gps_global_vel_up = row[18]

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
        gps_gyro_z[aa,:] = _gps_gyro_z
        gps_calc_vel_forward[aa,:] = (np.sin(_gps_heading_truth * np.pi/180)*_gps_global_vel_east + np.cos(_gps_heading_truth * np.pi/180)*_gps_global_vel_north)
        gps_calc_vel_lateral[aa,:] = (np.cos(_gps_heading_truth * np.pi/180)*_gps_global_vel_east + (-np.sin(_gps_heading_truth * np.pi/180)*_gps_global_vel_north))

    return  gps_time_truth, gps_northing_truth, gps_easting_truth, gps_heading_truth, gps_body_vel_x, gps_body_vel_y, gps_body_vel_z, gps_gyro_z, gps_calc_vel_forward, gps_calc_vel_lateral

def vehicle_data():
    args = parse_args() 
    vehicle = np.genfromtxt(args.vehicleData, skip_header=1, skip_footer=1, delimiter=' ')

    # parse vehicle data
    vn_accel_x = np.zeros((len(vehicle),1))
    vn_accel_y = np.zeros((len(vehicle),1))
    vn_accel_z = np.zeros((len(vehicle),1))
    avg_rear_axle_speed = np.zeros((len(vehicle),1))
    gps_time_vehicle = np.zeros((len(vehicle),1))
    gps_time_VN_vehicle = np.zeros((len(vehicle),1))
    steering_wheel_angle = np.zeros((len(vehicle),1))
    wheel_speed_fl = np.zeros((len(vehicle),1))
    wheel_speed_fr = np.zeros((len(vehicle),1))
    wheel_speed_rl = np.zeros((len(vehicle),1))
    wheel_speed_rr = np.zeros((len(vehicle),1))
    vn_yaw = np.zeros((len(vehicle),1))
    vn_pitch = np.zeros((len(vehicle),1))
    vn_roll = np.zeros((len(vehicle),1))
    vn_lat = np.zeros((len(vehicle),1))
    vn_lon = np.zeros((len(vehicle),1))
    vn_northing = np.zeros((len(vehicle),1))
    vn_easting = np.zeros((len(vehicle),1))

    ### time correction value for dealing with bad time reference ###
    gps_time_vehicle_correction = 0.0 # seconds

    # LLA to UTM conversion
    myProjTruth = Proj("+proj=utm +zone=10S, +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs")
    print('')
    print('converting truth lla to UTM...')

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
        _vn_yaw = row[7]
        _vn_pitch = row[8]
        _vn_roll = row[9]
        _gps_time_vn_vehicle = row[24]
        _vn_lat = row[16]
        _vn_lon = row[17]

        _utm_e_vn, _utm_n_vn = myProjTruth(_vn_lon,_vn_lat)
        vn_northing[i,:] = _utm_n_vn
        vn_easting[i,:] = _utm_e_vn 
        vn_accel_x[i,:] = _vn_accel_x
        vn_accel_y[i,:] = _vn_accel_y
        vn_accel_z[i,:] = _vn_accel_z
        vn_yaw[i,:] = _vn_yaw
        vn_pitch[i,:] = _vn_pitch
        vn_roll[i,:] = _vn_roll
        avg_rear_axle_speed[i,:] = (_wheel_speed_rl+_wheel_speed_rr)/2 # speed at center of rear axle
        gps_time_vehicle[i,:] = _gps_time_vehicle + gps_time_vehicle_correction
        gps_time_VN_vehicle[i,:] = _gps_time_vn_vehicle
        steering_wheel_angle[i,:] = _steer_wheel_angle
        wheel_speed_fl[i,:] = _wheel_speed_fl
        wheel_speed_fr[i,:] = _wheel_speed_fr
        wheel_speed_rl[i,:] = _wheel_speed_rl
        wheel_speed_rr[i,:] = _wheel_speed_rr
        vn_lat[i,:] = _vn_lat
        vn_lon[i,:] = _vn_lon

    return gps_time_vehicle, wheel_speed_fl, wheel_speed_fr, wheel_speed_rl, wheel_speed_rr, steering_wheel_angle, vn_accel_x, vn_accel_y, vn_accel_z, vn_yaw, vn_pitch, vn_roll, gps_time_VN_vehicle, vn_lat, vn_lon, vn_northing, vn_easting

def main():
    # tuning_values = tuning_honda()
    tuning_values = tuning_ford()
    vehicle_state = vehicle_data()
    novatel_state = novatel_data()

    sample_rate = 1/tuning_values[7]

    evaluation_gps_time = np.resize(vehicle_state[0], np.size(vehicle_state[0]))
    steering_wheel_angle = (np.resize(vehicle_state[5], np.size(vehicle_state[5])) + tuning_values[1]) * tuning_values[4]  
    avg_rear_axle_speed = np.resize(((vehicle_state[3] + vehicle_state[4]) * tuning_values[5] / 2), np.size(vehicle_state[0]))  # (rear left wheel speed + rear right wheel speed * speed scalar) / 2
    wheel_speed_fl = np.resize(vehicle_state[1], np.size(vehicle_state[1]))
    wheel_speed_fr = np.resize(vehicle_state[2], np.size(vehicle_state[2]))
    wheel_speed_rl = np.resize(vehicle_state[3], np.size(vehicle_state[3]))
    wheel_speed_rr = np.resize(vehicle_state[4], np.size(vehicle_state[4]))
    vn_pitch = np.resize(vehicle_state[10], np.size(vehicle_state[10]))
    vn_roll = np.resize(vehicle_state[11], np.size(vehicle_state[11]))
    vn_yaw = np.resize(vehicle_state[9], np.size(vehicle_state[9]))

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
    interp_vn_pitch = np.interp(novatel_state[0], evaluation_gps_time, vn_pitch)
    interp_vn_roll = np.interp(novatel_state[0], evaluation_gps_time, vn_roll)
    interp_vn_yaw = np.interp(novatel_state[0], evaluation_gps_time, vn_pitch)

    # relative novatel position
    gps_northing = novatel_state[1]
    gps_easting = novatel_state[2]
    relative_gps_northing = gps_northing - gps_northing[0]
    relative_gps_easting = gps_easting - gps_easting[0]

    # relative novatel heading
    gps_heading = novatel_state[3] * np.pi/180
    relative_gps_heading = gps_heading - gps_heading[0]

    ### variable steering ratio model calculation ###
    steering_ratio_speed_scalar = tuning_values[6]
    steering_rack_ratio = tuning_values[0]
    steering_ratio_variable = steering_rack_ratio + (interp_avg_rear_axle_speed * steering_ratio_speed_scalar)
    interp_steering_rack_angle_model = interp_steering_wheel_angle / steering_ratio_variable

    ### theta calculation ###
    theta = np.zeros((len(novatel_state[0]),1))
    theta_non_recursive = np.zeros((len(novatel_state[0]),1))
    theta_global = np.zeros((len(novatel_state[0]),1))
    theta_error_uncorrected = np.zeros((len(novatel_state[0]),1))
    theta_error_corrected = np.zeros((len(novatel_state[0]),1))
    relative_gps_heading_corrected = np.zeros((len(novatel_state[0]),1))
    zero = np.zeros((len(novatel_state[0]),1))
    wheel_speed_yaw_rate = np.zeros((len(novatel_state[0]),1))

    combined_data = np.column_stack((novatel_state[0], interp_avg_rear_axle_speed, 
                                    relative_gps_northing, relative_gps_easting, 
                                    relative_gps_heading, interp_steering_rack_angle_model,
                                    interp_wheel_speed_rl, interp_wheel_speed_rr))
    for i, row in enumerate(combined_data):
        _time = row[0]
        _speed = row[1]
        _rel_gps_northing = row[2]
        _rel_gps_easting = row[3]
        _rel_gps_heading = row[4]
        _rack_angle_model = row[5]
        _rear_left_speed = row[6]
        _rear_right_speed = row[7]

        _wheel_base = tuning_values[2]
        _track_width = tuning_values[3]
        _wheel_speed_scalar = tuning_values[5]
        _rl_speed_correction = tuning_values[10]

        ### rear wheel speed yaw rate estimate ###
        _diff_rear_speed = (_rear_right_speed - (_rear_left_speed * _rl_speed_correction)) * _wheel_speed_scalar
        wheel_speed_yaw_rate[i,:] = ((_diff_rear_speed / _track_width) * 180/np.pi) * -1

        ### sample rate ###
        dt = (i * sample_rate)

        ### theta ###
        theta[i,:] = (_speed/_wheel_base) * (np.tan(_rack_angle_model * np.pi/180)) * sample_rate + theta[i-1]
        theta_non_recursive[i,:] = (_speed/_wheel_base) * (np.tan(_rack_angle_model * np.pi/180)) * 180/np.pi * 1 ### UNSURE WHY SIGN FLIP IS NEEDED ONLY FOR HONDA!

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
        x_prediction[prediction_time,:] = interp_avg_rear_axle_speed[prediction_time] * np.cos(theta_global[prediction_time]) * sample_rate + x_prediction[prediction_time - 1]
        y_prediction[prediction_time,:] = interp_avg_rear_axle_speed[prediction_time] * np.sin(theta_global[prediction_time]) * sample_rate + y_prediction[prediction_time - 1]

    ### 2D pose error ###
    x_error = x_prediction - relative_gps_northing
    y_error = y_prediction - relative_gps_easting

    ### distance traveled ###
    distance_traveled_novatel = np.cumsum(novatel_state[5]) * sample_rate # scalar for sample rate
    distance_traveled_can = np.cumsum(interp_avg_rear_axle_speed) * sample_rate # scalar for sample rate
    distance_traveled_novatel = np.array(distance_traveled_novatel)
    distance_traveled_can = np.array(distance_traveled_can)
    distance_traveled_error = ((distance_traveled_novatel - distance_traveled_can) / distance_traveled_novatel) * 100.0

    ### yaw rate estimates ###
    novatel_yaw_rate = novatel_state[7]
    theta_global_rate = np.gradient(np.resize(theta_global, np.size(theta_global))) * (tuning_values[7]) * 180/np.pi # tuning_values[7] is the sample rate of the dataset
    theta_global_rate_resize = np.resize(theta_global_rate, (np.size(theta_global_rate),1)) # resizing to allow plotting
    theta_global_rate_error = (novatel_yaw_rate) - theta_global_rate_resize ### ONLY USED FOR FAKE NOVATEL DATA
    theta_global_rate_error_mean = np.mean(theta_global_rate_error)
    theta_global_rate_error_std = np.std(theta_global_rate_error)
    theta_global_rate_error_median = np.median(theta_global_rate_error)
    theta_global_rate_error_min = np.min(theta_global_rate_error)
    theta_global_rate_error_max = np.max(theta_global_rate_error)

    print()
    print(  'steering yaw rate estimate error:' )
    print(  ' mean: ', "{0:.4f}".format(theta_global_rate_error_mean), 
            ' std: ', "{0:.4f}".format(theta_global_rate_error_std), 
            ' median: ', "{0:.4f}".format(theta_global_rate_error_median), 
            ' min: ', "{0:.4f}".format(theta_global_rate_error_min), 
            ' max: ', "{0:.4f}".format(theta_global_rate_error_max))

    ### yaw rate estimates ###
    novatel_yaw_rate = novatel_state[7]
    ws_global_rate = wheel_speed_yaw_rate
    ws_global_rate_error = novatel_yaw_rate - wheel_speed_yaw_rate 
    ws_global_rate_error_mean = np.mean(ws_global_rate_error)
    ws_global_rate_error_std = np.std(ws_global_rate_error)
    ws_global_rate_error_median = np.median(ws_global_rate_error)
    ws_global_rate_error_min = np.min(ws_global_rate_error)
    ws_global_rate_error_max = np.max(ws_global_rate_error)

    print(  'rear wheel speed yaw rate estimate error:' )
    print(  ' mean: ', "{0:.4f}".format(ws_global_rate_error_mean), 
            ' std: ', "{0:.4f}".format(ws_global_rate_error_std), 
            ' median: ', "{0:.4f}".format(ws_global_rate_error_median), 
            ' min: ', "{0:.4f}".format(ws_global_rate_error_min), 
            ' max: ', "{0:.4f}".format(ws_global_rate_error_max))

    ### can latency statistics ###
    accel_error = interp_forward_accel - interp_calc_forward_accel
    accel_error_mean = np.mean(accel_error)
    accel_error_std = np.std(accel_error)
    accel_error_median = np.median(accel_error)
    accel_error_min = np.min(accel_error)
    accel_error_max = np.max(accel_error)
    
    ### encoder error stats ###
    # novatel_rotated_forward_velocity = novatel_state[8] ### Used for real novatel data
    novatel_rotated_forward_velocity = novatel_state[5] ### ONLY USED FOR FAKE NOVATEL DATA
    long_error = interp_avg_rear_axle_speed - novatel_rotated_forward_velocity
    long_error_mean = np.mean(long_error)
    long_error_std = np.std(long_error)
    long_error_median = np.median(long_error)
    long_error_min = np.min(long_error)
    long_error_max = np.max(long_error)

    print(  'encoder error:')
    print(  ' mean: ', "{0:.4f}".format(long_error_mean), 
            ' std: ', "{0:.4f}".format(long_error_std), 
            ' median: ', "{0:.4f}".format(long_error_median), 
            ' min: ', "{0:.4f}".format(long_error_min), 
            ' max: ', "{0:.4f}".format(long_error_max))

    ### plots ### 
    fig1, (ws, steer, error, theta, dist_trav_error, orient, gyro, gyro_error, sr, xy_error) = plt.subplots(10,1, sharex=True)
    fig1.suptitle('entire dataset')

    ws.plot(novatel_state[0], interp_wheel_speed_fl, label='fl wheel speed (m/s)')
    ws.plot(novatel_state[0], interp_wheel_speed_fr, label='fr wheel speed (m/s)')
    ws.plot(novatel_state[0], interp_wheel_speed_rl, label='rl wheel speed (m/s)')
    ws.plot(novatel_state[0], interp_wheel_speed_rr, label='rr wheel speed (m/s)')
    ws.plot(novatel_state[0], novatel_state[5], label='forward vel novatel (m/s)')
    ws.plot(novatel_state[0], novatel_state[4], label='body vel forward (m/s)')
    ws.legend()

    steer.plot(novatel_state[0], interp_steering_wheel_angle, label='steering wheel angle (deg)')
    steer.plot(novatel_state[0], interp_steering_rack_angle_model, label='steering rack angle (deg)')
    steer.legend()

    error.plot(novatel_state[0], theta_error_corrected * 180/np.pi, label='corrected theta error (deg)')
    error.plot(novatel_state[0], zero)
    error.legend()

    theta.plot(novatel_state[0], theta_global, label='theta global (rad)')
    theta.plot(novatel_state[0], gps_heading, label='global heading (rad)')
    theta.legend()

    dist_trav_error.plot(novatel_state[0], distance_traveled_error, label='percent error (%)')
    dist_trav_error.plot(novatel_state[0], zero)
    dist_trav_error.set_ylim(-1,1)
    dist_trav_error.legend()

    orient.plot(novatel_state[0], interp_vn_pitch, label='pitch (deg)')
    orient.plot(novatel_state[0], interp_vn_roll, label='roll (deg)')
    orient.legend()

    gyro.plot(novatel_state[0], novatel_yaw_rate, label='novatel yaw rate (deg/sec)')
    gyro.plot(novatel_state[0], theta_global_rate, label='theta rate (deg/sec)')
    gyro.plot(novatel_state[0], theta_non_recursive, label='theta non recursive (deg/sec)')
    gyro.legend()

    gyro_error.plot(novatel_state[0], theta_global_rate_error, label='yaw rate error (deg/sec)')
    gyro_error.legend()

    sr.plot(novatel_state[0], steering_ratio_variable, label='planned steering ratio term')
    sr.legend()

    xy_error.plot(novatel_state[0], x_error, label='x dead reckoning error (m)')
    xy_error.plot(novatel_state[0], y_error, label='y dead reckoning error (m)')
    xy_error.legend()

    plt.figure(2)
    plt.grid(b=True)
    plt.title('vehicle motion')
    plt.scatter(relative_gps_easting, relative_gps_northing, label='relative motion (utm)')
    plt.scatter(y_prediction, x_prediction, label='predicted path (m)')
    plt.legend()

    fig2, (vel, vel_error) = plt.subplots(2,1, sharex=True)
    fig2.suptitle('longitudinal error')
    vel.plot(novatel_state[0], interp_avg_rear_axle_speed, label='rear axle speed CAN (m/s)')
    vel.plot(novatel_state[0], novatel_state[5], label='forward velocity novatel (m/s)')
    vel.legend()

    # vel_error.plot(novatel_state[0], interp_avg_rear_axle_speed-novatel_state[8], label='forward velocity error (m/s)') ### Used with real novatel data
    vel_error.plot(novatel_state[0], interp_avg_rear_axle_speed-novatel_state[5], label='forward velocity error (m/s)') ### USED WITH FAKE NOVATEL DATA ONLY
    vel_error.legend()

    fig3, (yr, yr_error, vel_fw, vel_lat) = plt.subplots(4,1, sharex=True)
    fig3.suptitle('yaw rate estimation')
    yr.plot(novatel_state[0], theta_non_recursive, color='red', label='estimated yaw rate (deg/sec)')
    yr.plot(novatel_state[0], novatel_yaw_rate, color='green', label='novatel yaw rate (deg/sec)')
    yr.plot(novatel_state[0], wheel_speed_yaw_rate, color='blue', label='estimated wheel speed yaw rate (deg/sec)')
    yr.legend()

    yr_error.plot(novatel_state[0], theta_global_rate_error, label='yaw rate estimate error (deg/sec)')
    yr_error.plot(novatel_state[0], ws_global_rate_error, label='rear ws yaw rate estimate error (deg/sec)')
    yr_error.legend()

    vel_fw.plot(novatel_state[0], interp_avg_rear_axle_speed, label='avg rear axle speed (m/s)')
    vel_fw.plot(novatel_state[0], novatel_state[8], label='rotated forward vel novatel (m/s)')
    vel_fw.plot(novatel_state[0], novatel_state[4], label='body vel forward (m/s)')
    vel_fw.legend()

    vel_lat.plot(novatel_state[0], novatel_state[9], label='rotated lateral vel novatel (m/s)')
    vel_lat.plot(novatel_state[0], novatel_state[5], label='body vel lateral (m/s)')
    vel_lat.legend()

    plt.show()

if __name__=='__main__':
    main()

