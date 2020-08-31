#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
File: mag_compensation.py
Author: Tanja Baumann
Email: tanja@auterion.com
Github: https://github.com/baumanta
Description:
    Computes linear coefficients for mag compensation from thrust and current
Usage:
    python mag_compensation.py /path/to/log/logfile.ulg current --instance 1

Remark:
    If your logfile does not contain some of the topics, e.g.battery_status/current_a
    you will have to comment out the corresponding parts in the script
"""


import matplotlib.pylab as plt
from mpl_toolkits.mplot3d import Axes3D
from pyulog import ULog
from pyulog.px4 import PX4ULog
from pylab import *
import numpy as np
import textwrap as tw
import argparse

#arguments
parser = argparse.ArgumentParser(description='Calculate compensation parameters from ulog')

parser.add_argument('logfile', type=str, nargs='?', default=[],
                    help='full path to ulog file')
parser.add_argument('type', type=str, nargs='?', choices=['current', 'thrust'], default=[],
                    help='Power signal used for compensation, supported is "current" or "thrust".')
parser.add_argument('--instance', type=int, nargs='?', default=0,
                    help='instance of the current or thrust signal to use (0 or 1)')

args = parser.parse_args()
log_name = args.logfile
comp_type = args.type
comp_instance = args.instance

#Load the log data (produced by pyulog)
log = ULog(log_name)
pxlog = PX4ULog(log)

def get_data(topic_name, variable_name, index):
   try:
      dataset = log.get_dataset(topic_name, index)
      return dataset.data[variable_name]
   except:
      return []

def ms2s_list(time_ms_list):
    if len(time_ms_list) > 0:
        return 1e-6 * time_ms_list
    else:
        return time_ms_list

# Select msgs and copy into arrays
armed = get_data('vehicle_status', 'arming_state', 0)
t_armed =  ms2s_list(get_data('vehicle_status', 'timestamp', 0))

if comp_type == "thrust":
	power = get_data('vehicle_rates_setpoint', 'thrust_body[2]', comp_instance)
	power_t = ms2s_list(get_data('vehicle_rates_setpoint', 'timestamp', comp_instance))
	comp_type_param = 1
	factor = 1
	unit = "[G]"
elif comp_type == "current":
	power = get_data('battery_status', 'current_a', comp_instance)
	power = np.true_divide(power, 1000) #kA
	power_t = ms2s_list(get_data('battery_status', 'timestamp', comp_instance))
	comp_type_param = 2 + comp_instance
	factor = -1
	unit = "[G/kA]"
else:
	print("unknown compensation type {}. Supported is either 'thrust' or 'current'.".format(comp_type))
	sys.exit(1)

if len(power) == 0:
	print("could not retrieve power signal from log, zero data points")
	sys.exit(1)

mag0X_body = get_data('sensor_mag', 'x', 0)
mag0Y_body = get_data('sensor_mag', 'y', 0)
mag0Z_body = get_data('sensor_mag', 'z', 0)
t_mag0 =  ms2s_list(get_data('sensor_mag', 'timestamp', 0))
mag0_ID = get_data('sensor_mag', 'device_id', 0)

mag1X_body = get_data('sensor_mag', 'x', 1)
mag1Y_body = get_data('sensor_mag', 'y', 1)
mag1Z_body = get_data('sensor_mag', 'z', 1)
t_mag1 = ms2s_list(get_data('sensor_mag', 'timestamp', 1))
mag1_ID = get_data('sensor_mag', 'device_id', 1)

mag2X_body = get_data('sensor_mag', 'x', 2)
mag2Y_body = get_data('sensor_mag', 'y', 2)
mag2Z_body = get_data('sensor_mag', 'z', 2)
t_mag2 = ms2s_list(get_data('sensor_mag', 'timestamp', 2))
mag2_ID = get_data('sensor_mag', 'device_id', 2)

mag3X_body = get_data('sensor_mag', 'x', 3)
mag3Y_body = get_data('sensor_mag', 'y', 3)
mag3Z_body = get_data('sensor_mag', 'z', 3)
t_mag3 = ms2s_list(get_data('sensor_mag', 'timestamp', 3))
mag3_ID = get_data('sensor_mag', 'device_id', 3)

magX_body = []
magY_body = []
magZ_body = []
mag_id = []
t_mag = []

if len(mag0X_body) > 0:
    magX_body.append(mag0X_body)
    magY_body.append(mag0Y_body)
    magZ_body.append(mag0Z_body)
    t_mag.append(t_mag0)
    mag_id.append(mag0_ID[0])

if len(mag1X_body) > 0:
    magX_body.append(mag1X_body)
    magY_body.append(mag1Y_body)
    magZ_body.append(mag1Z_body)
    t_mag.append(t_mag1)
    mag_id.append(mag1_ID[0])

if len(mag2X_body) > 0:
    magX_body.append(mag2X_body)
    magY_body.append(mag2Y_body)
    magZ_body.append(mag2Z_body)
    t_mag.append(t_mag2)
    mag_id.append(mag2_ID[0])

if len(mag3X_body) > 0:
    magX_body.append(mag3X_body)
    magY_body.append(mag3Y_body)
    magZ_body.append(mag3Z_body)
    t_mag.append(t_mag3)
    mag_id.append(mag3_ID[0])

n_mag = len(magX_body)

#log index does not necessarily match mag calibration instance number
calibration_instance = []
instance_found = False
for idx in range(n_mag):
    instance_found = False
    for j in range(4):
        if mag_id[idx] == log.initial_parameters["CAL_MAG{}_ID".format(j)]:
                calibration_instance.append(j)
                instance_found = True
    if not instance_found:
        print('Mag {} calibration instance not found, run compass calibration first.'.format(mag_id[idx]))

#get first arming sequence from data
start_time = 0
stop_time = 0
for i in range(len(armed)-1):
    if armed[i] == 1 and armed[i+1] == 2:
        start_time = t_armed[i+1]
    if armed[i] == 2 and armed[i+1] == 1:
        stop_time = t_armed[i+1]
        break

#cut unarmed sequences from mag data
index_start = 0
index_stop = 0

for idx in range(n_mag):
    for i in range(len(t_mag[idx])):
        if t_mag[idx][i] > start_time:
            index_start = i
            break

    for i in range(len(t_mag[idx])):
        if t_mag[idx][i] > stop_time:
            index_stop = i -1
            break

    t_mag[idx] = t_mag[idx][index_start:index_stop]
    magX_body[idx] = magX_body[idx][index_start:index_stop]
    magY_body[idx] = magY_body[idx][index_start:index_stop]
    magZ_body[idx] = magZ_body[idx][index_start:index_stop]


#resample data
power_resampled = []

for idx in range(n_mag):
    power_resampled.append(interp(t_mag[idx], power_t, power))

#fit linear to get coefficients
px = []
py = []
pz = []

for idx in range(n_mag):
    px_temp, res_x, _, _, _ = polyfit(power_resampled[idx], magX_body[idx], 1,full = True)
    py_temp, res_y, _, _, _ = polyfit(power_resampled[idx], magY_body[idx], 1,full = True)
    pz_temp, res_z, _, _, _ = polyfit(power_resampled[idx], magZ_body[idx], 1, full = True)

    px.append(px_temp)
    py.append(py_temp)
    pz.append(pz_temp)

#print to console
for idx in range(n_mag):
    print('Mag{} device ID {} (calibration instance {})'.format(idx, mag_id[idx], calibration_instance[idx]))
print('\033[91m \n{}-based compensation: \033[0m'.format(comp_type))
print('\nparam set CAL_MAG_COMP_TYP {}'.format(comp_type_param))
for idx in range(n_mag):
    print('\nparam set CAL_MAG{}_XCOMP {:.3f}'.format(calibration_instance[idx], factor * px[idx][0]))
    print('param set CAL_MAG{}_YCOMP {:.3f}'.format(calibration_instance[idx], factor * py[idx][0]))
    print('param set CAL_MAG{}_ZCOMP {:.3f}'.format(calibration_instance[idx], factor * pz[idx][0]))

#plot data

for idx in range(n_mag):
    fig = plt.figure(num=None, figsize=(25, 14), dpi=80, facecolor='w', edgecolor='k')
    fig.suptitle('Compensation Parameter Fit \n{} \nmag {} ID: {} (calibration instance {})'.format(log_name, idx, mag_id[idx], calibration_instance[idx]), fontsize=14, fontweight='bold')

    plt.subplot(1,3,1)
    plt.plot(power_resampled[idx], magX_body[idx], 'yo', power_resampled[idx], px[idx][0]*power_resampled[idx]+px[idx][1], '--k')
    plt.xlabel('current [kA]')
    plt.ylabel('mag X [G]')

    plt.subplot(1,3,2)
    plt.plot(power_resampled[idx], magY_body[idx], 'yo', power_resampled[idx], py[idx][0]*power_resampled[idx]+py[idx][1], '--k')
    plt.xlabel('current [kA]')
    plt.ylabel('mag Y [G]')

    plt.subplot(1,3,3)
    plt.plot(power_resampled[idx], magZ_body[idx], 'yo', power_resampled[idx], pz[idx][0]*power_resampled[idx]+pz[idx][1], '--k')
    plt.xlabel('current [kA]')
    plt.ylabel('mag Z [G]')

    # display results
    plt.figtext(0.24, 0.03, 'CAL_MAG{}_XCOMP: {:.3f} {}'.format(calibration_instance[idx],factor * px[idx][0],unit), horizontalalignment='center', fontsize=12, multialignment='left', bbox=dict(boxstyle="round", facecolor='#D8D8D8', ec="0.5", pad=0.5, alpha=1), fontweight='bold')
    plt.figtext(0.51, 0.03, 'CAL_MAG{}_YCOMP: {:.3f} {}'.format(calibration_instance[idx],factor * py[idx][0],unit), horizontalalignment='center', fontsize=12, multialignment='left', bbox=dict(boxstyle="round", facecolor='#D8D8D8', ec="0.5", pad=0.5, alpha=1), fontweight='bold')
    plt.figtext(0.79, 0.03, 'CAL_MAG{}_ZCOMP: {:.3f} {}'.format(calibration_instance[idx],factor * pz[idx][0],unit), horizontalalignment='center', fontsize=12, multialignment='left', bbox=dict(boxstyle="round", facecolor='#D8D8D8', ec="0.5", pad=0.5, alpha=1), fontweight='bold')


#compensation comparison plots
for idx in range(n_mag):
    fig = plt.figure(num=None, figsize=(25, 14), dpi=80, facecolor='w', edgecolor='k')
    fig.suptitle('Original Data vs. Compensation \n{}\nmag {} ID: {} (calibration instance {})'.format(log_name, idx, mag_id[idx], calibration_instance[idx]), fontsize=14, fontweight='bold')

    plt.subplot(3,1,1)
    original_x, = plt.plot(t_mag[idx], magX_body[idx], label='original')
    power_x, = plt.plot(t_mag[idx],magX_body[idx] - px[idx][0] * power_resampled[idx], label='compensated')
    plt.legend(handles=[original_x, power_x])
    plt.xlabel('Time [s]')
    plt.ylabel('Mag X corrected[G]')

    plt.subplot(3,1,2)
    original_y, = plt.plot(t_mag[idx], magY_body[idx], label='original')
    power_y, = plt.plot(t_mag[idx],magY_body[idx] - py[idx][0] * power_resampled[idx], label='compensated')
    plt.legend(handles=[original_y, power_y])
    plt.xlabel('Time [s]')
    plt.ylabel('Mag Y corrected[G]')

    plt.subplot(3,1,3)
    original_z, = plt.plot(t_mag[idx], magZ_body[idx], label='original')
    power_z, = plt.plot(t_mag[idx],magZ_body[idx] - pz[idx][0] * power_resampled[idx], label='compensated')
    plt.legend(handles=[original_z, power_z])
    plt.xlabel('Time [s]')
    plt.ylabel('Mag Z corrected[G]')

plt.show()
