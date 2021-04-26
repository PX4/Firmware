#! /usr/bin/env python

from __future__ import print_function

import argparse
import os
import math
import matplotlib.pyplot as plt
import numpy as np

from pyulog import *

"""
Reads in IMU data from a static thermal calibration test and performs a curve fit of gyro, accel and baro bias vs temperature
Data can be gathered using the following sequence:

1) Power up the board and set the TC_A_ENABLE, TC_B_ENABLE and TC_G_ENABLE parameters to 1
2) Set all CAL_GYR and CAL_ACC parameters to defaults
3) Set the parameter SDLOG_MODE to 2, and SDLOG_PROFILE "Thermal calibration" bit (2) to enable logging of sensor data for calibration and power off
4) Cold soak the board for 30 minutes
5) Move to a warm dry, still air, constant pressure environment.
6) Apply power for 45 minutes, keeping the board still.
7) Remove power and extract the .ulog file
8) Open a terminal window in the Firmware/Tools directory and run the python calibration script script file: 'python process_sensor_caldata.py <full path name to .ulog file>
9) Power the board, connect QGC and load the parameter from the generated .params file onto the board using QGC. Due to the number of parameters, loading them may take some time.
10) TODO - we need a way for user to reliably tell when parameters have all been changed and saved.
11) After parameters have finished loading, set SDLOG_MODE and SDLOG_PROFILE to their respective values prior to step 4) and remove power.
12) Power the board and perform a normal gyro and accelerometer sensor calibration using QGC. The board must be repowered after this step before flying due to large parameter changes and the thermal compensation parameters only being read on startup.

Outputs thermal compensation parameters in a file named <inputfilename>.params which can be loaded onto the board using QGroundControl
Outputs summary plots in a pdf file named <inputfilename>.pdf

"""

def resampleWithDeltaX(x,y):
    xMin = np.amin(x)
    xMax = np.amax(x)
    nbInterval = 2000
    interval = (xMax-xMin)/nbInterval

    resampledY  = np.zeros(nbInterval)
    resampledX  = np.zeros(nbInterval)
    resampledCount = np.zeros(nbInterval)

    for idx in range(0,len(x)):
        if x[idx]<xMin:
            binIdx = 0
        elif x[idx]<xMax:
            binIdx = int((x[idx]-xMin)/(interval))
        else:
            binIdx = nbInterval-1
        resampledY[binIdx] += y[idx]
        resampledX[binIdx] += x[idx]
        resampledCount[binIdx] += 1

    idxNotEmpty = np.where(resampledCount != 0)
    resampledCount = resampledCount[idxNotEmpty]
    resampledY = resampledY[idxNotEmpty]
    resampledX = resampledX[idxNotEmpty]

    resampledY /= resampledCount
    resampledX /= resampledCount

    return resampledX,resampledY

parser = argparse.ArgumentParser(description='Reads in IMU data from a static thermal calibration test and performs a curve fit of gyro, accel and baro bias vs temperature')
parser.add_argument('filename', metavar='file.ulg', help='ULog input file')
parser.add_argument('--no_resample', dest='noResample', action='store_const',
                   const=True, default=False, help='skip resampling and use raw data')

def is_valid_directory(parser, arg):
    if os.path.isdir(arg):
        # Directory exists so return the directory
        return arg
    else:
        parser.error('The directory {} does not exist'.format(arg))

args = parser.parse_args()
ulog_file_name = args.filename
noResample = args.noResample

ulog = ULog(ulog_file_name, None)
data = ulog.data_list

# extract gyro data
num_gyros = 0
for d in data:
    if d.name == 'sensor_gyro':
        if d.multi_id == 0:
            sensor_gyro_0 = d.data
            print('found gyro 0 data')
            num_gyros += 1
        elif d.multi_id == 1:
            sensor_gyro_1 = d.data
            print('found gyro 1 data')
            num_gyros += 1
        elif d.multi_id == 2:
            sensor_gyro_2 = d.data
            print('found gyro 2 data')
            num_gyros += 1
        elif d.multi_id == 3:
            sensor_gyro_3 = d.data
            print('found gyro 3 data')
            num_gyros += 1

# extract accel data
num_accels = 0
for d in data:
    if d.name == 'sensor_accel':
        if d.multi_id == 0:
            sensor_accel_0 = d.data
            print('found accel 0 data')
            num_accels += 1
        elif d.multi_id == 1:
            sensor_accel_1 = d.data
            print('found accel 1 data')
            num_accels += 1
        elif d.multi_id == 2:
            sensor_accel_2 = d.data
            print('found accel 2 data')
            num_accels += 1
        elif d.multi_id == 3:
            sensor_accel_3 = d.data
            print('found accel 3 data')
            num_accels += 1

# extract baro data
num_baros = 0
for d in data:
    if d.name == 'sensor_baro':
        if d.multi_id == 0:
            sensor_baro_0 = d.data
            print('found baro 0 data')
            num_baros += 1
        elif d.multi_id == 1:
            sensor_baro_1 = d.data
            print('found baro 1 data')
            num_baros += 1
        elif d.multi_id == 2:
            sensor_baro_2 = d.data
            print('found baro 2 data')
            num_baros += 1
        elif d.multi_id == 3:
            sensor_baro_3 = d.data
            print('found baro 3 data')
            num_baros += 1

# open file to save plots to PDF
from matplotlib.backends.backend_pdf import PdfPages
output_plot_filename = ulog_file_name + ".pdf"
pp = PdfPages(output_plot_filename)

#################################################################################

# define data dictionary of gyro 0 thermal correction  parameters
gyro_0_params = {
'TC_G0_ID':0,
'TC_G0_TMIN':0.0,
'TC_G0_TMAX':0.0,
'TC_G0_TREF':0.0,
'TC_G0_X0_0':0.0,
'TC_G0_X1_0':0.0,
'TC_G0_X2_0':0.0,
'TC_G0_X3_0':0.0,
'TC_G0_X0_1':0.0,
'TC_G0_X1_1':0.0,
'TC_G0_X2_1':0.0,
'TC_G0_X3_1':0.0,
'TC_G0_X0_2':0.0,
'TC_G0_X1_2':0.0,
'TC_G0_X2_2':0.0,
'TC_G0_X3_2':0.0
}

# curve fit the data for gyro 0 corrections
if num_gyros >= 1 and not math.isnan(sensor_gyro_0['temperature'][0]):
    gyro_0_params['TC_G0_ID'] = int(np.median(sensor_gyro_0['device_id']))

    # find the min, max and reference temperature
    gyro_0_params['TC_G0_TMIN'] = np.amin(sensor_gyro_0['temperature'])
    gyro_0_params['TC_G0_TMAX'] = np.amax(sensor_gyro_0['temperature'])
    gyro_0_params['TC_G0_TREF'] = 0.5 * (gyro_0_params['TC_G0_TMIN'] + gyro_0_params['TC_G0_TMAX'])
    temp_rel = sensor_gyro_0['temperature'] - gyro_0_params['TC_G0_TREF']
    temp_rel_resample = np.linspace(gyro_0_params['TC_G0_TMIN']-gyro_0_params['TC_G0_TREF'], gyro_0_params['TC_G0_TMAX']-gyro_0_params['TC_G0_TREF'], 100)
    temp_resample = temp_rel_resample + gyro_0_params['TC_G0_TREF']

    # fit X axis
    if noResample:
        coef_gyro_0_x = np.polyfit(temp_rel,sensor_gyro_0['x'],3)
    else:
        temp, sens = resampleWithDeltaX(temp_rel,sensor_gyro_0['x'])
        coef_gyro_0_x = np.polyfit(temp, sens ,3)

    gyro_0_params['TC_G0_X3_0'] = coef_gyro_0_x[0]
    gyro_0_params['TC_G0_X2_0'] = coef_gyro_0_x[1]
    gyro_0_params['TC_G0_X1_0'] = coef_gyro_0_x[2]
    gyro_0_params['TC_G0_X0_0'] = coef_gyro_0_x[3]
    fit_coef_gyro_0_x = np.poly1d(coef_gyro_0_x)
    gyro_0_x_resample = fit_coef_gyro_0_x(temp_rel_resample)

    # fit Y axis
    if noResample:
        coef_gyro_0_y = np.polyfit(temp_rel,sensor_gyro_0['y'],3)
    else:
        temp, sens = resampleWithDeltaX(temp_rel,sensor_gyro_0['y'])
        coef_gyro_0_y = np.polyfit(temp, sens ,3)

    gyro_0_params['TC_G0_X3_1'] = coef_gyro_0_y[0]
    gyro_0_params['TC_G0_X2_1'] = coef_gyro_0_y[1]
    gyro_0_params['TC_G0_X1_1'] = coef_gyro_0_y[2]
    gyro_0_params['TC_G0_X0_1'] = coef_gyro_0_y[3]
    fit_coef_gyro_0_y = np.poly1d(coef_gyro_0_y)
    gyro_0_y_resample = fit_coef_gyro_0_y(temp_rel_resample)

    # fit Z axis
    if noResample:
        coef_gyro_0_z = np.polyfit(temp_rel,sensor_gyro_0['z'],3)
    else:
        temp, sens = resampleWithDeltaX(temp_rel,sensor_gyro_0['z'])
        coef_gyro_0_z = np.polyfit(temp, sens ,3)

    gyro_0_params['TC_G0_X3_2'] = coef_gyro_0_z[0]
    gyro_0_params['TC_G0_X2_2'] = coef_gyro_0_z[1]
    gyro_0_params['TC_G0_X1_2'] = coef_gyro_0_z[2]
    gyro_0_params['TC_G0_X0_2'] = coef_gyro_0_z[3]
    fit_coef_gyro_0_z = np.poly1d(coef_gyro_0_z)
    gyro_0_z_resample = fit_coef_gyro_0_z(temp_rel_resample)

    # gyro0 vs temperature
    plt.figure(1,figsize=(20,13))

    # draw plots
    plt.subplot(3,1,1)
    plt.plot(sensor_gyro_0['temperature'],sensor_gyro_0['x'],'b')
    plt.plot(temp_resample,gyro_0_x_resample,'r')
    plt.title('Gyro 0 ({}) Bias vs Temperature'.format(gyro_0_params['TC_G0_ID']))
    plt.ylabel('X bias (rad/s)')
    plt.xlabel('temperature (degC)')
    plt.grid()

    # draw plots
    plt.subplot(3,1,2)
    plt.plot(sensor_gyro_0['temperature'],sensor_gyro_0['y'],'b')
    plt.plot(temp_resample,gyro_0_y_resample,'r')
    plt.ylabel('Y bias (rad/s)')
    plt.xlabel('temperature (degC)')
    plt.grid()

    # draw plots
    plt.subplot(3,1,3)
    plt.plot(sensor_gyro_0['temperature'],sensor_gyro_0['z'],'b')
    plt.plot(temp_resample,gyro_0_z_resample,'r')
    plt.ylabel('Z bias (rad/s)')
    plt.xlabel('temperature (degC)')
    plt.grid()

    pp.savefig()

#################################################################################

#################################################################################

# define data dictionary of gyro 1 thermal correction  parameters
gyro_1_params = {
'TC_G1_ID':0,
'TC_G1_TMIN':0.0,
'TC_G1_TMAX':0.0,
'TC_G1_TREF':0.0,
'TC_G1_X0_0':0.0,
'TC_G1_X1_0':0.0,
'TC_G1_X2_0':0.0,
'TC_G1_X3_0':0.0,
'TC_G1_X0_1':0.0,
'TC_G1_X1_1':0.0,
'TC_G1_X2_1':0.0,
'TC_G1_X3_1':0.0,
'TC_G1_X0_2':0.0,
'TC_G1_X1_2':0.0,
'TC_G1_X2_2':0.0,
'TC_G1_X3_2':0.0
}

# curve fit the data for gyro 1 corrections
if num_gyros >= 2 and not math.isnan(sensor_gyro_1['temperature'][0]):
    gyro_1_params['TC_G1_ID'] = int(np.median(sensor_gyro_1['device_id']))

    # find the min, max and reference temperature
    gyro_1_params['TC_G1_TMIN'] = np.amin(sensor_gyro_1['temperature'])
    gyro_1_params['TC_G1_TMAX'] = np.amax(sensor_gyro_1['temperature'])
    gyro_1_params['TC_G1_TREF'] = 0.5 * (gyro_1_params['TC_G1_TMIN'] + gyro_1_params['TC_G1_TMAX'])
    temp_rel = sensor_gyro_1['temperature'] - gyro_1_params['TC_G1_TREF']
    temp_rel_resample = np.linspace(gyro_1_params['TC_G1_TMIN']-gyro_1_params['TC_G1_TREF'], gyro_1_params['TC_G1_TMAX']-gyro_1_params['TC_G1_TREF'], 100)
    temp_resample = temp_rel_resample + gyro_1_params['TC_G1_TREF']

    # fit X axis
    if noResample:
        coef_gyro_1_x = np.polyfit(temp_rel,sensor_gyro_1['x'],3)
    else:
        temp, sens = resampleWithDeltaX(temp_rel,sensor_gyro_1['x'])
        coef_gyro_1_x = np.polyfit(temp, sens ,3)

    gyro_1_params['TC_G1_X3_0'] = coef_gyro_1_x[0]
    gyro_1_params['TC_G1_X2_0'] = coef_gyro_1_x[1]
    gyro_1_params['TC_G1_X1_0'] = coef_gyro_1_x[2]
    gyro_1_params['TC_G1_X0_0'] = coef_gyro_1_x[3]
    fit_coef_gyro_1_x = np.poly1d(coef_gyro_1_x)
    gyro_1_x_resample = fit_coef_gyro_1_x(temp_rel_resample)

    # fit Y axis
    if noResample:
        coef_gyro_1_y = np.polyfit(temp_rel,sensor_gyro_1['y'],3)
    else:
        temp, sens = resampleWithDeltaX(temp_rel,sensor_gyro_1['y'])
        coef_gyro_1_y = np.polyfit(temp, sens ,3)

    gyro_1_params['TC_G1_X3_1'] = coef_gyro_1_y[0]
    gyro_1_params['TC_G1_X2_1'] = coef_gyro_1_y[1]
    gyro_1_params['TC_G1_X1_1'] = coef_gyro_1_y[2]
    gyro_1_params['TC_G1_X0_1'] = coef_gyro_1_y[3]
    fit_coef_gyro_1_y = np.poly1d(coef_gyro_1_y)
    gyro_1_y_resample = fit_coef_gyro_1_y(temp_rel_resample)

    # fit Z axis
    if noResample:
        coef_gyro_1_z = np.polyfit(temp_rel,sensor_gyro_1['z'],3)
    else:
        temp, sens = resampleWithDeltaX(temp_rel,sensor_gyro_1['z'])
        coef_gyro_1_z = np.polyfit(temp, sens ,3)

    gyro_1_params['TC_G1_X3_2'] = coef_gyro_1_z[0]
    gyro_1_params['TC_G1_X2_2'] = coef_gyro_1_z[1]
    gyro_1_params['TC_G1_X1_2'] = coef_gyro_1_z[2]
    gyro_1_params['TC_G1_X0_2'] = coef_gyro_1_z[3]
    fit_coef_gyro_1_z = np.poly1d(coef_gyro_1_z)
    gyro_1_z_resample = fit_coef_gyro_1_z(temp_rel_resample)

    # gyro1 vs temperature
    plt.figure(2,figsize=(20,13))

    # draw plots
    plt.subplot(3,1,1)
    plt.plot(sensor_gyro_1['temperature'],sensor_gyro_1['x'],'b')
    plt.plot(temp_resample,gyro_1_x_resample,'r')
    plt.title('Gyro 1 ({}) Bias vs Temperature'.format(gyro_1_params['TC_G1_ID']))
    plt.ylabel('X bias (rad/s)')
    plt.xlabel('temperature (degC)')
    plt.grid()

    # draw plots
    plt.subplot(3,1,2)
    plt.plot(sensor_gyro_1['temperature'],sensor_gyro_1['y'],'b')
    plt.plot(temp_resample,gyro_1_y_resample,'r')
    plt.ylabel('Y bias (rad/s)')
    plt.xlabel('temperature (degC)')
    plt.grid()

    # draw plots
    plt.subplot(3,1,3)
    plt.plot(sensor_gyro_1['temperature'],sensor_gyro_1['z'],'b')
    plt.plot(temp_resample,gyro_1_z_resample,'r')
    plt.ylabel('Z bias (rad/s)')
    plt.xlabel('temperature (degC)')
    plt.grid()

    pp.savefig()

#################################################################################

#################################################################################

# define data dictionary of gyro 2 thermal correction  parameters
gyro_2_params = {
'TC_G2_ID':0,
'TC_G2_TMIN':0.0,
'TC_G2_TMAX':0.0,
'TC_G2_TREF':0.0,
'TC_G2_X0_0':0.0,
'TC_G2_X1_0':0.0,
'TC_G2_X2_0':0.0,
'TC_G2_X3_0':0.0,
'TC_G2_X0_1':0.0,
'TC_G2_X1_1':0.0,
'TC_G2_X2_1':0.0,
'TC_G2_X3_1':0.0,
'TC_G2_X0_2':0.0,
'TC_G2_X1_2':0.0,
'TC_G2_X2_2':0.0,
'TC_G2_X3_2':0.0
}

# curve fit the data for gyro 2 corrections
if num_gyros >= 3 and not math.isnan(sensor_gyro_2['temperature'][0]):
    gyro_2_params['TC_G2_ID'] = int(np.median(sensor_gyro_2['device_id']))

    # find the min, max and reference temperature
    gyro_2_params['TC_G2_TMIN'] = np.amin(sensor_gyro_2['temperature'])
    gyro_2_params['TC_G2_TMAX'] = np.amax(sensor_gyro_2['temperature'])
    gyro_2_params['TC_G2_TREF'] = 0.5 * (gyro_2_params['TC_G2_TMIN'] + gyro_2_params['TC_G2_TMAX'])
    temp_rel = sensor_gyro_2['temperature'] - gyro_2_params['TC_G2_TREF']
    temp_rel_resample = np.linspace(gyro_2_params['TC_G2_TMIN']-gyro_2_params['TC_G2_TREF'], gyro_2_params['TC_G2_TMAX']-gyro_2_params['TC_G2_TREF'], 100)
    temp_resample = temp_rel_resample + gyro_2_params['TC_G2_TREF']

    # fit X axis
    if noResample:
        coef_gyro_2_x = np.polyfit(temp_rel,sensor_gyro_2['x'],3)
    else:
        temp, sens = resampleWithDeltaX(temp_rel,sensor_gyro_2['x'])
        coef_gyro_2_x = np.polyfit(temp, sens ,3)

    gyro_2_params['TC_G2_X3_0'] = coef_gyro_2_x[0]
    gyro_2_params['TC_G2_X2_0'] = coef_gyro_2_x[1]
    gyro_2_params['TC_G2_X1_0'] = coef_gyro_2_x[2]
    gyro_2_params['TC_G2_X0_0'] = coef_gyro_2_x[3]
    fit_coef_gyro_2_x = np.poly1d(coef_gyro_2_x)
    gyro_2_x_resample = fit_coef_gyro_2_x(temp_rel_resample)

    # fit Y axis
    if noResample:
        coef_gyro_2_y = np.polyfit(temp_rel,sensor_gyro_2['y'],3)
    else:
        temp, sens = resampleWithDeltaX(temp_rel,sensor_gyro_2['y'])
        coef_gyro_2_y = np.polyfit(temp, sens ,3)

    gyro_2_params['TC_G2_X3_1'] = coef_gyro_2_y[0]
    gyro_2_params['TC_G2_X2_1'] = coef_gyro_2_y[1]
    gyro_2_params['TC_G2_X1_1'] = coef_gyro_2_y[2]
    gyro_2_params['TC_G2_X0_1'] = coef_gyro_2_y[3]
    fit_coef_gyro_2_y = np.poly1d(coef_gyro_2_y)
    gyro_2_y_resample = fit_coef_gyro_2_y(temp_rel_resample)

    # fit Z axis
    if noResample:
        coef_gyro_2_z = np.polyfit(temp_rel,sensor_gyro_2['z'],3)
    else:
        temp, sens = resampleWithDeltaX(temp_rel,sensor_gyro_2['z'])
        coef_gyro_2_z = np.polyfit(temp, sens ,3)

    gyro_2_params['TC_G2_X3_2'] = coef_gyro_2_z[0]
    gyro_2_params['TC_G2_X2_2'] = coef_gyro_2_z[1]
    gyro_2_params['TC_G2_X1_2'] = coef_gyro_2_z[2]
    gyro_2_params['TC_G2_X0_2'] = coef_gyro_2_z[3]
    fit_coef_gyro_2_z = np.poly1d(coef_gyro_2_z)
    gyro_2_z_resample = fit_coef_gyro_2_z(temp_rel_resample)

    # gyro2 vs temperature
    plt.figure(3,figsize=(20,13))

    # draw plots
    plt.subplot(3,1,1)
    plt.plot(sensor_gyro_2['temperature'],sensor_gyro_2['x'],'b')
    plt.plot(temp_resample,gyro_2_x_resample,'r')
    plt.title('Gyro 2 ({}) Bias vs Temperature'.format(gyro_2_params['TC_G2_ID']))
    plt.ylabel('X bias (rad/s)')
    plt.xlabel('temperature (degC)')
    plt.grid()

    # draw plots
    plt.subplot(3,1,2)
    plt.plot(sensor_gyro_2['temperature'],sensor_gyro_2['y'],'b')
    plt.plot(temp_resample,gyro_2_y_resample,'r')
    plt.ylabel('Y bias (rad/s)')
    plt.xlabel('temperature (degC)')
    plt.grid()

    # draw plots
    plt.subplot(3,1,3)
    plt.plot(sensor_gyro_2['temperature'],sensor_gyro_2['z'],'b')
    plt.plot(temp_resample,gyro_2_z_resample,'r')
    plt.ylabel('Z bias (rad/s)')
    plt.xlabel('temperature (degC)')
    plt.grid()

    pp.savefig()

#################################################################################

#################################################################################

# define data dictionary of gyro 3 thermal correction  parameters
gyro_3_params = {
'TC_G3_ID':0,
'TC_G3_TMIN':0.0,
'TC_G3_TMAX':0.0,
'TC_G3_TREF':0.0,
'TC_G3_X0_0':0.0,
'TC_G3_X1_0':0.0,
'TC_G3_X2_0':0.0,
'TC_G3_X3_0':0.0,
'TC_G3_X0_1':0.0,
'TC_G3_X1_1':0.0,
'TC_G3_X2_1':0.0,
'TC_G3_X3_1':0.0,
'TC_G3_X0_2':0.0,
'TC_G3_X1_2':0.0,
'TC_G3_X2_2':0.0,
'TC_G3_X3_2':0.0
}

# curve fit the data for gyro 3 corrections
if num_gyros >= 4 and not math.isnan(sensor_gyro_3['temperature'][0]):
    gyro_3_params['TC_G3_ID'] = int(np.median(sensor_gyro_3['device_id']))

    # find the min, max and reference temperature
    gyro_3_params['TC_G3_TMIN'] = np.amin(sensor_gyro_3['temperature'])
    gyro_3_params['TC_G3_TMAX'] = np.amax(sensor_gyro_3['temperature'])
    gyro_3_params['TC_G3_TREF'] = 0.5 * (gyro_3_params['TC_G3_TMIN'] + gyro_3_params['TC_G3_TMAX'])
    temp_rel = sensor_gyro_3['temperature'] - gyro_3_params['TC_G3_TREF']
    temp_rel_resample = np.linspace(gyro_3_params['TC_G3_TMIN']-gyro_3_params['TC_G3_TREF'], gyro_3_params['TC_G3_TMAX']-gyro_3_params['TC_G3_TREF'], 100)
    temp_resample = temp_rel_resample + gyro_3_params['TC_G3_TREF']

    # fit X axis
    coef_gyro_3_x = np.polyfit(temp_rel,sensor_gyro_3['x'],3)
    gyro_3_params['TC_G3_X3_0'] = coef_gyro_3_x[0]
    gyro_3_params['TC_G3_X2_0'] = coef_gyro_3_x[1]
    gyro_3_params['TC_G3_X1_0'] = coef_gyro_3_x[2]
    gyro_3_params['TC_G3_X0_0'] = coef_gyro_3_x[3]
    fit_coef_gyro_3_x = np.poly1d(coef_gyro_3_x)
    gyro_3_x_resample = fit_coef_gyro_3_x(temp_rel_resample)

    # fit Y axis
    coef_gyro_3_y = np.polyfit(temp_rel,sensor_gyro_3['y'],3)
    gyro_3_params['TC_G3_X3_1'] = coef_gyro_3_y[0]
    gyro_3_params['TC_G3_X2_1'] = coef_gyro_3_y[1]
    gyro_3_params['TC_G3_X1_1'] = coef_gyro_3_y[2]
    gyro_3_params['TC_G3_X0_1'] = coef_gyro_3_y[3]
    fit_coef_gyro_3_y = np.poly1d(coef_gyro_3_y)
    gyro_3_y_resample = fit_coef_gyro_3_y(temp_rel_resample)

    # fit Z axis
    coef_gyro_3_z = np.polyfit(temp_rel,sensor_gyro_3['z'],3)
    gyro_3_params['TC_G3_X3_2'] = coef_gyro_3_z[0]
    gyro_3_params['TC_G3_X2_2'] = coef_gyro_3_z[1]
    gyro_3_params['TC_G3_X1_2'] = coef_gyro_3_z[2]
    gyro_3_params['TC_G3_X0_2'] = coef_gyro_3_z[3]
    fit_coef_gyro_3_z = np.poly1d(coef_gyro_3_z)
    gyro_3_z_resample = fit_coef_gyro_3_z(temp_rel_resample)

    # gyro3 vs temperature
    plt.figure(4,figsize=(20,13))

    # draw plots
    plt.subplot(3,1,1)
    plt.plot(sensor_gyro_3['temperature'],sensor_gyro_3['x'],'b')
    plt.plot(temp_resample,gyro_3_x_resample,'r')
    plt.title('Gyro 2 ({}) Bias vs Temperature'.format(gyro_3_params['TC_G3_ID']))
    plt.ylabel('X bias (rad/s)')
    plt.xlabel('temperature (degC)')
    plt.grid()

    # draw plots
    plt.subplot(3,1,2)
    plt.plot(sensor_gyro_3['temperature'],sensor_gyro_3['y'],'b')
    plt.plot(temp_resample,gyro_3_y_resample,'r')
    plt.ylabel('Y bias (rad/s)')
    plt.xlabel('temperature (degC)')
    plt.grid()

    # draw plots
    plt.subplot(3,1,3)
    plt.plot(sensor_gyro_3['temperature'],sensor_gyro_3['z'],'b')
    plt.plot(temp_resample,gyro_3_z_resample,'r')
    plt.ylabel('Z bias (rad/s)')
    plt.xlabel('temperature (degC)')
    plt.grid()

    pp.savefig()

#################################################################################

#################################################################################

# define data dictionary of accel 0 thermal correction  parameters
accel_0_params = {
'TC_A0_ID':0,
'TC_A0_TMIN':0.0,
'TC_A0_TMAX':0.0,
'TC_A0_TREF':0.0,
'TC_A0_X0_0':0.0,
'TC_A0_X1_0':0.0,
'TC_A0_X2_0':0.0,
'TC_A0_X3_0':0.0,
'TC_A0_X0_1':0.0,
'TC_A0_X1_1':0.0,
'TC_A0_X2_1':0.0,
'TC_A0_X3_1':0.0,
'TC_A0_X0_2':0.0,
'TC_A0_X1_2':0.0,
'TC_A0_X2_2':0.0,
'TC_A0_X3_2':0.0
}

# curve fit the data for accel 0 corrections
if num_accels >= 1 and not math.isnan(sensor_accel_0['temperature'][0]):
    accel_0_params['TC_A0_ID'] = int(np.median(sensor_accel_0['device_id']))

    # find the min, max and reference temperature
    accel_0_params['TC_A0_TMIN'] = np.amin(sensor_accel_0['temperature'])
    accel_0_params['TC_A0_TMAX'] = np.amax(sensor_accel_0['temperature'])
    accel_0_params['TC_A0_TREF'] = 0.5 * (accel_0_params['TC_A0_TMIN'] + accel_0_params['TC_A0_TMAX'])
    temp_rel = sensor_accel_0['temperature'] - accel_0_params['TC_A0_TREF']
    temp_rel_resample = np.linspace(accel_0_params['TC_A0_TMIN']-accel_0_params['TC_A0_TREF'], accel_0_params['TC_A0_TMAX']-accel_0_params['TC_A0_TREF'], 100)
    temp_resample = temp_rel_resample + accel_0_params['TC_A0_TREF']

    # fit X axis
    correction_x = sensor_accel_0['x'] - np.median(sensor_accel_0['x'])
    if noResample:
        coef_accel_0_x = np.polyfit(temp_rel,correction_x,3)
    else:
        temp, sens = resampleWithDeltaX(temp_rel,correction_x)
        coef_accel_0_x = np.polyfit(temp, sens ,3)

    accel_0_params['TC_A0_X3_0'] = coef_accel_0_x[0]
    accel_0_params['TC_A0_X2_0'] = coef_accel_0_x[1]
    accel_0_params['TC_A0_X1_0'] = coef_accel_0_x[2]
    accel_0_params['TC_A0_X0_0'] = coef_accel_0_x[3]
    fit_coef_accel_0_x = np.poly1d(coef_accel_0_x)
    correction_x_resample = fit_coef_accel_0_x(temp_rel_resample)

    # fit Y axis
    correction_y = sensor_accel_0['y']-np.median(sensor_accel_0['y'])
    if noResample:
        coef_accel_0_y = np.polyfit(temp_rel,correction_y,3)
    else:
        temp, sens = resampleWithDeltaX(temp_rel,correction_y)
        coef_accel_0_y = np.polyfit(temp, sens ,3)

    accel_0_params['TC_A0_X3_1'] = coef_accel_0_y[0]
    accel_0_params['TC_A0_X2_1'] = coef_accel_0_y[1]
    accel_0_params['TC_A0_X1_1'] = coef_accel_0_y[2]
    accel_0_params['TC_A0_X0_1'] = coef_accel_0_y[3]
    fit_coef_accel_0_y = np.poly1d(coef_accel_0_y)
    correction_y_resample = fit_coef_accel_0_y(temp_rel_resample)

    # fit Z axis
    correction_z = sensor_accel_0['z']-np.median(sensor_accel_0['z'])
    if noResample:
        coef_accel_0_z = np.polyfit(temp_rel,correction_z,3)
    else:
        temp, sens = resampleWithDeltaX(temp_rel,correction_z)
        coef_accel_0_z = np.polyfit(temp, sens ,3)

    accel_0_params['TC_A0_X3_2'] = coef_accel_0_z[0]
    accel_0_params['TC_A0_X2_2'] = coef_accel_0_z[1]
    accel_0_params['TC_A0_X1_2'] = coef_accel_0_z[2]
    accel_0_params['TC_A0_X0_2'] = coef_accel_0_z[3]
    fit_coef_accel_0_z = np.poly1d(coef_accel_0_z)
    correction_z_resample = fit_coef_accel_0_z(temp_rel_resample)

    # accel 0 vs temperature
    plt.figure(5,figsize=(20,13))

    # draw plots
    plt.subplot(3,1,1)
    plt.plot(sensor_accel_0['temperature'],correction_x,'b')
    plt.plot(temp_resample,correction_x_resample,'r')
    plt.title('Accel 0 ({}) Bias vs Temperature'.format(accel_0_params['TC_A0_ID']))
    plt.ylabel('X bias (m/s/s)')
    plt.xlabel('temperature (degC)')
    plt.grid()

    # draw plots
    plt.subplot(3,1,2)
    plt.plot(sensor_accel_0['temperature'],correction_y,'b')
    plt.plot(temp_resample,correction_y_resample,'r')
    plt.ylabel('Y bias (m/s/s)')
    plt.xlabel('temperature (degC)')
    plt.grid()

    # draw plots
    plt.subplot(3,1,3)
    plt.plot(sensor_accel_0['temperature'],correction_z,'b')
    plt.plot(temp_resample,correction_z_resample,'r')
    plt.ylabel('Z bias (m/s/s)')
    plt.xlabel('temperature (degC)')
    plt.grid()

    pp.savefig()

#################################################################################

#################################################################################

# define data dictionary of accel 1 thermal correction  parameters
accel_1_params = {
'TC_A1_ID':0,
'TC_A1_TMIN':0.0,
'TC_A1_TMAX':0.0,
'TC_A1_TREF':0.0,
'TC_A1_X0_0':0.0,
'TC_A1_X1_0':0.0,
'TC_A1_X2_0':0.0,
'TC_A1_X3_0':0.0,
'TC_A1_X0_1':0.0,
'TC_A1_X1_1':0.0,
'TC_A1_X2_1':0.0,
'TC_A1_X3_1':0.0,
'TC_A1_X0_2':0.0,
'TC_A1_X1_2':0.0,
'TC_A1_X2_2':0.0,
'TC_A1_X3_2':0.0
}

# curve fit the data for accel 1 corrections
if num_accels >= 2 and not math.isnan(sensor_accel_1['temperature'][0]):
    accel_1_params['TC_A1_ID'] = int(np.median(sensor_accel_1['device_id']))

    # find the min, max and reference temperature
    accel_1_params['TC_A1_TMIN'] = np.amin(sensor_accel_1['temperature'])
    accel_1_params['TC_A1_TMAX'] = np.amax(sensor_accel_1['temperature'])
    accel_1_params['TC_A1_TREF'] = 0.5 * (accel_1_params['TC_A1_TMIN'] + accel_1_params['TC_A1_TMAX'])
    temp_rel = sensor_accel_1['temperature'] - accel_1_params['TC_A1_TREF']
    temp_rel_resample = np.linspace(accel_1_params['TC_A1_TMIN']-accel_1_params['TC_A1_TREF'], accel_1_params['TC_A1_TMAX']-accel_1_params['TC_A1_TREF'], 100)
    temp_resample = temp_rel_resample + accel_1_params['TC_A1_TREF']

    # fit X axis
    correction_x = sensor_accel_1['x']-np.median(sensor_accel_1['x'])
    if noResample:
        coef_accel_1_x = np.polyfit(temp_rel,correction_x,3)
    else:
        temp, sens = resampleWithDeltaX(temp_rel,correction_x)
        coef_accel_1_x = np.polyfit(temp, sens ,3)

    accel_1_params['TC_A1_X3_0'] = coef_accel_1_x[0]
    accel_1_params['TC_A1_X2_0'] = coef_accel_1_x[1]
    accel_1_params['TC_A1_X1_0'] = coef_accel_1_x[2]
    accel_1_params['TC_A1_X0_0'] = coef_accel_1_x[3]
    fit_coef_accel_1_x = np.poly1d(coef_accel_1_x)
    correction_x_resample = fit_coef_accel_1_x(temp_rel_resample)

    # fit Y axis
    correction_y = sensor_accel_1['y']-np.median(sensor_accel_1['y'])
    if noResample:
        coef_accel_1_y = np.polyfit(temp_rel,correction_y,3)
    else:
        temp, sens = resampleWithDeltaX(temp_rel,correction_y)
        coef_accel_1_y = np.polyfit(temp, sens ,3)

    accel_1_params['TC_A1_X3_1'] = coef_accel_1_y[0]
    accel_1_params['TC_A1_X2_1'] = coef_accel_1_y[1]
    accel_1_params['TC_A1_X1_1'] = coef_accel_1_y[2]
    accel_1_params['TC_A1_X0_1'] = coef_accel_1_y[3]
    fit_coef_accel_1_y = np.poly1d(coef_accel_1_y)
    correction_y_resample = fit_coef_accel_1_y(temp_rel_resample)

    # fit Z axis
    correction_z = (sensor_accel_1['z'])-np.median(sensor_accel_1['z'])
    if noResample:
        coef_accel_1_z = np.polyfit(temp_rel,correction_z,3)
    else:
        temp, sens = resampleWithDeltaX(temp_rel,correction_z)
        coef_accel_1_z = np.polyfit(temp, sens ,3)

    accel_1_params['TC_A1_X3_2'] = coef_accel_1_z[0]
    accel_1_params['TC_A1_X2_2'] = coef_accel_1_z[1]
    accel_1_params['TC_A1_X1_2'] = coef_accel_1_z[2]
    accel_1_params['TC_A1_X0_2'] = coef_accel_1_z[3]
    fit_coef_accel_1_z = np.poly1d(coef_accel_1_z)
    correction_z_resample = fit_coef_accel_1_z(temp_rel_resample)

    # accel 1 vs temperature
    plt.figure(6,figsize=(20,13))

    # draw plots
    plt.subplot(3,1,1)
    plt.plot(sensor_accel_1['temperature'],correction_x,'b')
    plt.plot(temp_resample,correction_x_resample,'r')
    plt.title('Accel 1 ({}) Bias vs Temperature'.format(accel_1_params['TC_A1_ID']))
    plt.ylabel('X bias (m/s/s)')
    plt.xlabel('temperature (degC)')
    plt.grid()

    # draw plots
    plt.subplot(3,1,2)
    plt.plot(sensor_accel_1['temperature'],correction_y,'b')
    plt.plot(temp_resample,correction_y_resample,'r')
    plt.ylabel('Y bias (m/s/s)')
    plt.xlabel('temperature (degC)')
    plt.grid()

    # draw plots
    plt.subplot(3,1,3)
    plt.plot(sensor_accel_1['temperature'],correction_z,'b')
    plt.plot(temp_resample,correction_z_resample,'r')
    plt.ylabel('Z bias (m/s/s)')
    plt.xlabel('temperature (degC)')
    plt.grid()

    pp.savefig()


#################################################################################

#################################################################################

# define data dictionary of accel 2 thermal correction  parameters
accel_2_params = {
'TC_A2_ID':0,
'TC_A2_TMIN':0.0,
'TC_A2_TMAX':0.0,
'TC_A2_TREF':0.0,
'TC_A2_X0_0':0.0,
'TC_A2_X1_0':0.0,
'TC_A2_X2_0':0.0,
'TC_A2_X3_0':0.0,
'TC_A2_X0_1':0.0,
'TC_A2_X1_1':0.0,
'TC_A2_X2_1':0.0,
'TC_A2_X3_1':0.0,
'TC_A2_X0_2':0.0,
'TC_A2_X1_2':0.0,
'TC_A2_X2_2':0.0,
'TC_A2_X3_2':0.0
}

# curve fit the data for accel 2 corrections
if num_accels >= 3 and not math.isnan(sensor_accel_2['temperature'][0]):
    accel_2_params['TC_A2_ID'] = int(np.median(sensor_accel_2['device_id']))

    # find the min, max and reference temperature
    accel_2_params['TC_A2_TMIN'] = np.amin(sensor_accel_2['temperature'])
    accel_2_params['TC_A2_TMAX'] = np.amax(sensor_accel_2['temperature'])
    accel_2_params['TC_A2_TREF'] = 0.5 * (accel_2_params['TC_A2_TMIN'] + accel_2_params['TC_A2_TMAX'])
    temp_rel = sensor_accel_2['temperature'] - accel_2_params['TC_A2_TREF']
    temp_rel_resample = np.linspace(accel_2_params['TC_A2_TMIN']-accel_2_params['TC_A2_TREF'], accel_2_params['TC_A2_TMAX']-accel_2_params['TC_A2_TREF'], 100)
    temp_resample = temp_rel_resample + accel_2_params['TC_A2_TREF']

    # fit X axis
    correction_x = sensor_accel_2['x']-np.median(sensor_accel_2['x'])
    if noResample:
        coef_accel_2_x = np.polyfit(temp_rel,correction_x,3)
    else:
        temp, sens = resampleWithDeltaX(temp_rel,correction_x)
        coef_accel_2_x = np.polyfit(temp, sens ,3)

    accel_2_params['TC_A2_X3_0'] = coef_accel_2_x[0]
    accel_2_params['TC_A2_X2_0'] = coef_accel_2_x[1]
    accel_2_params['TC_A2_X1_0'] = coef_accel_2_x[2]
    accel_2_params['TC_A2_X0_0'] = coef_accel_2_x[3]
    fit_coef_accel_2_x = np.poly1d(coef_accel_2_x)
    correction_x_resample = fit_coef_accel_2_x(temp_rel_resample)

    # fit Y axis
    correction_y = sensor_accel_2['y']-np.median(sensor_accel_2['y'])
    if noResample:
        coef_accel_2_y = np.polyfit(temp_rel,correction_y,3)
    else:
        temp, sens = resampleWithDeltaX(temp_rel,correction_y)
        coef_accel_2_y = np.polyfit(temp, sens ,3)

    accel_2_params['TC_A2_X3_1'] = coef_accel_2_y[0]
    accel_2_params['TC_A2_X2_1'] = coef_accel_2_y[1]
    accel_2_params['TC_A2_X1_1'] = coef_accel_2_y[2]
    accel_2_params['TC_A2_X0_1'] = coef_accel_2_y[3]
    fit_coef_accel_2_y = np.poly1d(coef_accel_2_y)
    correction_y_resample = fit_coef_accel_2_y(temp_rel_resample)

    # fit Z axis
    correction_z = sensor_accel_2['z']-np.median(sensor_accel_2['z'])
    if noResample:
        coef_accel_2_z = np.polyfit(temp_rel,correction_z,3)
    else:
        temp, sens = resampleWithDeltaX(temp_rel,correction_z)
        coef_accel_2_z = np.polyfit(temp, sens ,3)

    accel_2_params['TC_A2_X3_2'] = coef_accel_2_z[0]
    accel_2_params['TC_A2_X2_2'] = coef_accel_2_z[1]
    accel_2_params['TC_A2_X1_2'] = coef_accel_2_z[2]
    accel_2_params['TC_A2_X0_2'] = coef_accel_2_z[3]
    fit_coef_accel_2_z = np.poly1d(coef_accel_2_z)
    correction_z_resample = fit_coef_accel_2_z(temp_rel_resample)

    # accel 2 vs temperature
    plt.figure(7,figsize=(20,13))

    # draw plots
    plt.subplot(3,1,1)
    plt.plot(sensor_accel_2['temperature'],correction_x,'b')
    plt.plot(temp_resample,correction_x_resample,'r')
    plt.title('Accel 2 ({}) Bias vs Temperature'.format(accel_2_params['TC_A2_ID']))
    plt.ylabel('X bias (m/s/s)')
    plt.xlabel('temperature (degC)')
    plt.grid()

    # draw plots
    plt.subplot(3,1,2)
    plt.plot(sensor_accel_2['temperature'],correction_y,'b')
    plt.plot(temp_resample,correction_y_resample,'r')
    plt.ylabel('Y bias (m/s/s)')
    plt.xlabel('temperature (degC)')
    plt.grid()

    # draw plots
    plt.subplot(3,1,3)
    plt.plot(sensor_accel_2['temperature'],correction_z,'b')
    plt.plot(temp_resample,correction_z_resample,'r')
    plt.ylabel('Z bias (m/s/s)')
    plt.xlabel('temperature (degC)')
    plt.grid()

    pp.savefig()

#################################################################################

#################################################################################

# define data dictionary of accel 3 thermal correction  parameters
accel_3_params = {
'TC_A3_ID':0,
'TC_A3_TMIN':0.0,
'TC_A3_TMAX':0.0,
'TC_A3_TREF':0.0,
'TC_A3_X0_0':0.0,
'TC_A3_X1_0':0.0,
'TC_A3_X2_0':0.0,
'TC_A3_X3_0':0.0,
'TC_A3_X0_1':0.0,
'TC_A3_X1_1':0.0,
'TC_A3_X2_1':0.0,
'TC_A3_X3_1':0.0,
'TC_A3_X0_2':0.0,
'TC_A3_X1_2':0.0,
'TC_A3_X2_2':0.0,
'TC_A3_X3_2':0.0
}

# curve fit the data for accel 2 corrections
if num_accels >= 4 and not math.isnan(sensor_accel_3['temperature'][0]):
    accel_3_params['TC_A3_ID'] = int(np.median(sensor_accel_3['device_id']))

    # find the min, max and reference temperature
    accel_3_params['TC_A3_TMIN'] = np.amin(sensor_accel_3['temperature'])
    accel_3_params['TC_A3_TMAX'] = np.amax(sensor_accel_3['temperature'])
    accel_3_params['TC_A3_TREF'] = 0.5 * (accel_3_params['TC_A3_TMIN'] + accel_3_params['TC_A3_TMAX'])
    temp_rel = sensor_accel_3['temperature'] - accel_3_params['TC_A3_TREF']
    temp_rel_resample = np.linspace(accel_3_params['TC_A3_TMIN']-accel_3_params['TC_A3_TREF'], accel_3_params['TC_A3_TMAX']-accel_3_params['TC_A3_TREF'], 100)
    temp_resample = temp_rel_resample + accel_3_params['TC_A3_TREF']

    # fit X axis
    correction_x = sensor_accel_3['x']-np.median(sensor_accel_3['x'])
    coef_accel_3_x = np.polyfit(temp_rel,correction_x,3)
    accel_3_params['TC_A3_X3_0'] = coef_accel_3_x[0]
    accel_3_params['TC_A3_X2_0'] = coef_accel_3_x[1]
    accel_3_params['TC_A3_X1_0'] = coef_accel_3_x[2]
    accel_3_params['TC_A3_X0_0'] = coef_accel_3_x[3]
    fit_coef_accel_3_x = np.poly1d(coef_accel_3_x)
    correction_x_resample = fit_coef_accel_3_x(temp_rel_resample)

    # fit Y axis
    correction_y = sensor_accel_3['y']-np.median(sensor_accel_3['y'])
    coef_accel_3_y = np.polyfit(temp_rel,correction_y,3)
    accel_3_params['TC_A3_X3_1'] = coef_accel_3_y[0]
    accel_3_params['TC_A3_X2_1'] = coef_accel_3_y[1]
    accel_3_params['TC_A3_X1_1'] = coef_accel_3_y[2]
    accel_3_params['TC_A3_X0_1'] = coef_accel_3_y[3]
    fit_coef_accel_3_y = np.poly1d(coef_accel_3_y)
    correction_y_resample = fit_coef_accel_3_y(temp_rel_resample)

    # fit Z axis
    correction_z = sensor_accel_3['z']-np.median(sensor_accel_3['z'])
    coef_accel_3_z = np.polyfit(temp_rel,correction_z,3)
    accel_3_params['TC_A3_X3_2'] = coef_accel_3_z[0]
    accel_3_params['TC_A3_X2_2'] = coef_accel_3_z[1]
    accel_3_params['TC_A3_X1_2'] = coef_accel_3_z[2]
    accel_3_params['TC_A3_X0_2'] = coef_accel_3_z[3]
    fit_coef_accel_3_z = np.poly1d(coef_accel_3_z)
    correction_z_resample = fit_coef_accel_3_z(temp_rel_resample)

    # accel 3 vs temperature
    plt.figure(8,figsize=(20,13))

    # draw plots
    plt.subplot(3,1,1)
    plt.plot(sensor_accel_3['temperature'],correction_x,'b')
    plt.plot(temp_resample,correction_x_resample,'r')
    plt.title('Accel 3 ({}) Bias vs Temperature'.format(accel_3_params['TC_A3_ID']))
    plt.ylabel('X bias (m/s/s)')
    plt.xlabel('temperature (degC)')
    plt.grid()

    # draw plots
    plt.subplot(3,1,2)
    plt.plot(sensor_accel_3['temperature'],correction_y,'b')
    plt.plot(temp_resample,correction_y_resample,'r')
    plt.ylabel('Y bias (m/s/s)')
    plt.xlabel('temperature (degC)')
    plt.grid()

    # draw plots
    plt.subplot(3,1,3)
    plt.plot(sensor_accel_3['temperature'],correction_z,'b')
    plt.plot(temp_resample,correction_z_resample,'r')
    plt.ylabel('Z bias (m/s/s)')
    plt.xlabel('temperature (degC)')
    plt.grid()

    pp.savefig()

#################################################################################

#################################################################################

# define data dictionary of baro 0 thermal correction  parameters
baro_0_params = {
'TC_B0_ID':0,
'TC_B0_TMIN':0.0,
'TC_B0_TMAX':0.0,
'TC_B0_TREF':0.0,
'TC_B0_X0':0.0,
'TC_B0_X1':0.0,
'TC_B0_X2':0.0,
'TC_B0_X3':0.0,
'TC_B0_X4':0.0,
'TC_B0_X5':0.0
}

# curve fit the data for baro 0 corrections
baro_0_params['TC_B0_ID'] = int(np.median(sensor_baro_0['device_id']))

# find the min, max and reference temperature
baro_0_params['TC_B0_TMIN'] = np.amin(sensor_baro_0['temperature'])
baro_0_params['TC_B0_TMAX'] = np.amax(sensor_baro_0['temperature'])
baro_0_params['TC_B0_TREF'] = 0.5 * (baro_0_params['TC_B0_TMIN'] + baro_0_params['TC_B0_TMAX'])
temp_rel = sensor_baro_0['temperature'] - baro_0_params['TC_B0_TREF']
temp_rel_resample = np.linspace(baro_0_params['TC_B0_TMIN']-baro_0_params['TC_B0_TREF'], baro_0_params['TC_B0_TMAX']-baro_0_params['TC_B0_TREF'], 100)
temp_resample = temp_rel_resample + baro_0_params['TC_B0_TREF']

# fit data
median_pressure = np.median(sensor_baro_0['pressure']);
if noResample:
    coef_baro_0_x = np.polyfit(temp_rel,100*(sensor_baro_0['pressure']-median_pressure),5) # convert from hPa to Pa
else:
    temperature, baro = resampleWithDeltaX(temp_rel,100*(sensor_baro_0['pressure']-median_pressure)) # convert from hPa to Pa
    coef_baro_0_x = np.polyfit(temperature,baro,5)

baro_0_params['TC_B0_X5'] = coef_baro_0_x[0]
baro_0_params['TC_B0_X4'] = coef_baro_0_x[1]
baro_0_params['TC_B0_X3'] = coef_baro_0_x[2]
baro_0_params['TC_B0_X2'] = coef_baro_0_x[3]
baro_0_params['TC_B0_X1'] = coef_baro_0_x[4]
baro_0_params['TC_B0_X0'] = coef_baro_0_x[5]
fit_coef_baro_0_x = np.poly1d(coef_baro_0_x)
baro_0_x_resample = fit_coef_baro_0_x(temp_rel_resample)

# baro 0 vs temperature
plt.figure(9,figsize=(20,13))

# draw plots
plt.plot(sensor_baro_0['temperature'],100*sensor_baro_0['pressure']-100*median_pressure,'b')
plt.plot(temp_resample,baro_0_x_resample,'r')
plt.title('Baro 0 ({}) Bias vs Temperature'.format(baro_0_params['TC_B0_ID']))
plt.ylabel('Z bias (Pa)')
plt.xlabel('temperature (degC)')
plt.grid()

pp.savefig()

# define data dictionary of baro 1 thermal correction  parameters
baro_1_params = {
'TC_B1_ID':0,
'TC_B1_TMIN':0.0,
'TC_B1_TMAX':0.0,
'TC_B1_TREF':0.0,
'TC_B1_X0':0.0,
'TC_B1_X1':0.0,
'TC_B1_X2':0.0,
'TC_B1_X3':0.0,
'TC_B1_X4':0.0,
'TC_B1_X5':0.0,
}

if num_baros >= 2:

    # curve fit the data for baro 1 corrections
    baro_1_params['TC_B1_ID'] = int(np.median(sensor_baro_1['device_id']))

    # find the min, max and reference temperature
    baro_1_params['TC_B1_TMIN'] = np.amin(sensor_baro_1['temperature'])
    baro_1_params['TC_B1_TMAX'] = np.amax(sensor_baro_1['temperature'])
    baro_1_params['TC_B1_TREF'] = 0.5 * (baro_1_params['TC_B1_TMIN'] + baro_1_params['TC_B1_TMAX'])
    temp_rel = sensor_baro_1['temperature'] - baro_1_params['TC_B1_TREF']
    temp_rel_resample = np.linspace(baro_1_params['TC_B1_TMIN']-baro_1_params['TC_B1_TREF'], baro_1_params['TC_B1_TMAX']-baro_1_params['TC_B1_TREF'], 100)
    temp_resample = temp_rel_resample + baro_1_params['TC_B1_TREF']

    # fit data
    median_pressure = np.median(sensor_baro_1['pressure']);
    if noResample:
        coef_baro_1_x = np.polyfit(temp_rel,100*(sensor_baro_1['pressure']-median_pressure),5) # convert from hPa to Pa
    else:
        temperature, baro = resampleWithDeltaX(temp_rel,100*(sensor_baro_1['pressure']-median_pressure)) # convert from hPa to Pa
        coef_baro_1_x = np.polyfit(temperature,baro,5)

    baro_1_params['TC_B1_X5'] = coef_baro_1_x[0]
    baro_1_params['TC_B1_X4'] = coef_baro_1_x[1]
    baro_1_params['TC_B1_X3'] = coef_baro_1_x[2]
    baro_1_params['TC_B1_X2'] = coef_baro_1_x[3]
    baro_1_params['TC_B1_X1'] = coef_baro_1_x[4]
    baro_1_params['TC_B1_X0'] = coef_baro_1_x[5]
    fit_coef_baro_1_x = np.poly1d(coef_baro_1_x)
    baro_1_x_resample = fit_coef_baro_1_x(temp_rel_resample)

    # baro 2 vs temperature
    plt.figure(10,figsize=(20,13))

    # draw plots
    plt.plot(sensor_baro_1['temperature'],100*sensor_baro_1['pressure']-100*median_pressure,'b')
    plt.plot(temp_resample,baro_1_x_resample,'r')
    plt.title('Baro 1 ({}) Bias vs Temperature'.format(baro_1_params['TC_B1_ID']))
    plt.ylabel('Z bias (Pa)')
    plt.xlabel('temperature (degC)')
    plt.grid()

    pp.savefig()

# define data dictionary of baro 2 thermal correction  parameters
baro_2_params = {
'TC_B2_ID':0,
'TC_B2_TMIN':0.0,
'TC_B2_TMAX':0.0,
'TC_B2_TREF':0.0,
'TC_B2_X0':0.0,
'TC_B2_X1':0.0,
'TC_B2_X2':0.0,
'TC_B2_X3':0.0,
'TC_B2_X4':0.0,
'TC_B2_X5':0.0,
'TC_B2_SCL':1.0,
}

if num_baros >= 3:

    # curve fit the data for baro 2 corrections
    baro_2_params['TC_B2_ID'] = int(np.median(sensor_baro_2['device_id']))

    # find the min, max and reference temperature
    baro_2_params['TC_B2_TMIN'] = np.amin(sensor_baro_2['temperature'])
    baro_2_params['TC_B2_TMAX'] = np.amax(sensor_baro_2['temperature'])
    baro_2_params['TC_B2_TREF'] = 0.5 * (baro_2_params['TC_B2_TMIN'] + baro_2_params['TC_B2_TMAX'])
    temp_rel = sensor_baro_2['temperature'] - baro_2_params['TC_B2_TREF']
    temp_rel_resample = np.linspace(baro_2_params['TC_B2_TMIN']-baro_2_params['TC_B2_TREF'], baro_2_params['TC_B2_TMAX']-baro_2_params['TC_B2_TREF'], 100)
    temp_resample = temp_rel_resample + baro_2_params['TC_B2_TREF']

    # fit data
    median_pressure = np.median(sensor_baro_2['pressure']);
    if noResample:
        coef_baro_2_x = np.polyfit(temp_rel,100*(sensor_baro_2['pressure']-median_pressure),5) # convert from hPa to Pa
    else:
        temperature, baro = resampleWithDeltaX(temp_rel,100*(sensor_baro_2['pressure']-median_pressure)) # convert from hPa to Pa
        coef_baro_2_x = np.polyfit(temperature,baro,5)

    baro_2_params['TC_B2_X5'] = coef_baro_2_x[0]
    baro_2_params['TC_B2_X4'] = coef_baro_2_x[1]
    baro_2_params['TC_B2_X3'] = coef_baro_2_x[2]
    baro_2_params['TC_B2_X2'] = coef_baro_2_x[3]
    baro_2_params['TC_B2_X1'] = coef_baro_2_x[4]
    baro_2_params['TC_B2_X0'] = coef_baro_2_x[5]
    fit_coef_baro_2_x = np.poly1d(coef_baro_2_x)
    baro_2_x_resample = fit_coef_baro_2_x(temp_rel_resample)

    # baro 2 vs temperature
    plt.figure(11,figsize=(20,13))

    # draw plots
    plt.plot(sensor_baro_2['temperature'],100*sensor_baro_2['pressure']-100*median_pressure,'b')
    plt.plot(temp_resample,baro_2_x_resample,'r')
    plt.title('Baro 2 ({}) Bias vs Temperature'.format(baro_2_params['TC_B2_ID']))
    plt.ylabel('Z bias (Pa)')
    plt.xlabel('temperature (degC)')
    plt.grid()

    pp.savefig()

# define data dictionary of baro 3 thermal correction  parameters
baro_3_params = {
'TC_B3_ID':0,
'TC_B3_TMIN':0.0,
'TC_B3_TMAX':0.0,
'TC_B3_TREF':0.0,
'TC_B3_X0':0.0,
'TC_B3_X1':0.0,
'TC_B3_X2':0.0,
'TC_B3_X3':0.0,
'TC_B3_X4':0.0,
'TC_B3_X5':0.0,
'TC_B3_SCL':1.0,
}

if num_baros >= 4:

    # curve fit the data for baro 2 corrections
    baro_3_params['TC_B3_ID'] = int(np.median(sensor_baro_3['device_id']))

    # find the min, max and reference temperature
    baro_3_params['TC_B3_TMIN'] = np.amin(sensor_baro_3['temperature'])
    baro_3_params['TC_B3_TMAX'] = np.amax(sensor_baro_3['temperature'])
    baro_3_params['TC_B3_TREF'] = 0.5 * (baro_3_params['TC_B3_TMIN'] + baro_3_params['TC_B3_TMAX'])
    temp_rel = sensor_baro_3['temperature'] - baro_3_params['TC_B3_TREF']
    temp_rel_resample = np.linspace(baro_3_params['TC_B3_TMIN']-baro_3_params['TC_B3_TREF'], baro_3_params['TC_B3_TMAX']-baro_3_params['TC_B3_TREF'], 100)
    temp_resample = temp_rel_resample + baro_3_params['TC_B3_TREF']

    # fit data
    median_pressure = np.median(sensor_baro_3['pressure'])
    coef_baro_3_x = np.polyfit(temp_rel,100*(sensor_baro_3['pressure']-median_pressure),5) # convert from hPa to Pa
    baro_3_params['TC_B3_X5'] = coef_baro_3_x[0]
    baro_3_params['TC_B3_X4'] = coef_baro_3_x[1]
    baro_3_params['TC_B3_X3'] = coef_baro_3_x[2]
    baro_3_params['TC_B3_X2'] = coef_baro_3_x[3]
    baro_3_params['TC_B3_X1'] = coef_baro_3_x[4]
    baro_3_params['TC_B3_X0'] = coef_baro_3_x[5]
    fit_coef_baro_3_x = np.poly1d(coef_baro_3_x)
    baro_3_x_resample = fit_coef_baro_3_x(temp_rel_resample)

    # baro 3 vs temperature
    plt.figure(12,figsize=(20,13))

    # draw plots
    plt.plot(sensor_baro_3['temperature'],100*sensor_baro_3['pressure']-100*median_pressure,'b')
    plt.plot(temp_resample,baro_3_x_resample,'r')
    plt.title('Baro 3 ({}) Bias vs Temperature'.format(baro_3_params['TC_B3_ID']))
    plt.ylabel('Z bias (Pa)')
    plt.xlabel('temperature (degC)')
    plt.grid()

    pp.savefig()

#################################################################################

# close the pdf file
pp.close()

# clase all figures
plt.close("all")

# write correction parameters to file
test_results_filename = ulog_file_name + ".params"
file = open(test_results_filename,"w")
file.write("# Sensor thermal compensation parameters\n")
file.write("#\n")
file.write("# Vehicle-Id Component-Id Name Value Type\n")

# accel 0 corrections
key_list_accel = list(accel_0_params.keys())
key_list_accel.sort
for key in key_list_accel:
    if key == 'TC_A0_ID':
        type = "6"
    else:
        type = "9"
    file.write("1"+"\t"+"1"+"\t"+key+"\t"+str(accel_0_params[key])+"\t"+type+"\n")

# accel 1 corrections
key_list_accel = list(accel_1_params.keys())
key_list_accel.sort
for key in key_list_accel:
    if key == 'TC_A1_ID':
        type = "6"
    else:
        type = "9"
    file.write("1"+"\t"+"1"+"\t"+key+"\t"+str(accel_1_params[key])+"\t"+type+"\n")

# accel 2 corrections
key_list_accel = list(accel_2_params.keys())
key_list_accel.sort
for key in key_list_accel:
    if key == 'TC_A2_ID':
        type = "6"
    else:
        type = "9"
    file.write("1"+"\t"+"1"+"\t"+key+"\t"+str(accel_2_params[key])+"\t"+type+"\n")

# accel 3 corrections
key_list_accel = list(accel_3_params.keys())
key_list_accel.sort
for key in key_list_accel:
    if key == 'TC_A3_ID':
        type = "6"
    else:
        type = "9"
    file.write("1"+"\t"+"1"+"\t"+key+"\t"+str(accel_3_params[key])+"\t"+type+"\n")

# baro 0 corrections
key_list_baro = list(baro_0_params.keys())
key_list_baro.sort
for key in key_list_baro:
    if key == 'TC_B0_ID':
        type = "6"
    else:
        type = "9"
    file.write("1"+"\t"+"1"+"\t"+key+"\t"+str(baro_0_params[key])+"\t"+type+"\n")

# baro 1 corrections
key_list_baro = list(baro_1_params.keys())
key_list_baro.sort
for key in key_list_baro:
    if key == 'TC_B1_ID':
        type = "6"
    else:
        type = "9"
    file.write("1"+"\t"+"1"+"\t"+key+"\t"+str(baro_1_params[key])+"\t"+type+"\n")

# baro 2 corrections
key_list_baro = list(baro_2_params.keys())
key_list_baro.sort
for key in key_list_baro:
    if key == 'TC_B2_ID':
        type = "6"
    else:
        type = "9"
    file.write("1"+"\t"+"1"+"\t"+key+"\t"+str(baro_2_params[key])+"\t"+type+"\n")

# baro 3 corrections
key_list_baro = list(baro_3_params.keys())
key_list_baro.sort
for key in key_list_baro:
    if key == 'TC_B3_ID':
        type = "6"
    else:
        type = "9"
    file.write("1"+"\t"+"1"+"\t"+key+"\t"+str(baro_3_params[key])+"\t"+type+"\n")


# gyro 0 corrections
key_list_gyro = list(gyro_0_params.keys())
key_list_gyro.sort()
for key in key_list_gyro:
    if key == 'TC_G0_ID':
        type = "6"
    else:
        type = "9"
    file.write("1"+"\t"+"1"+"\t"+key+"\t"+str(gyro_0_params[key])+"\t"+type+"\n")

# gyro 1 corrections
key_list_gyro = list(gyro_1_params.keys())
key_list_gyro.sort()
for key in key_list_gyro:
    if key == 'TC_G1_ID':
        type = "6"
    else:
        type = "9"
    file.write("1"+"\t"+"1"+"\t"+key+"\t"+str(gyro_1_params[key])+"\t"+type+"\n")

# gyro 2 corrections
key_list_gyro = list(gyro_2_params.keys())
key_list_gyro.sort()
for key in key_list_gyro:
    if key == 'TC_G2_ID':
        type = "6"
    else:
        type = "9"
    file.write("1"+"\t"+"1"+"\t"+key+"\t"+str(gyro_2_params[key])+"\t"+type+"\n")

# gyro 3 corrections
key_list_gyro = list(gyro_3_params.keys())
key_list_gyro.sort()
for key in key_list_gyro:
    if key == 'TC_G3_ID':
        type = "6"
    else:
        type = "9"
    file.write("1"+"\t"+"1"+"\t"+key+"\t"+str(gyro_3_params[key])+"\t"+type+"\n")

file.close()

print('Correction parameters written to ' + test_results_filename)
print('Plots saved to ' + output_plot_filename)
