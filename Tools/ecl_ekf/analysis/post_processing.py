#! /usr/bin/env python3
"""
function collection for post-processing of ulog data.
"""

from typing import Tuple

import numpy as np


def get_estimator_check_flags(estimator_status: dict) -> Tuple[dict, dict, dict]:
    """
    :param estimator_status:
    :return:
    """
    control_mode = get_control_mode_flags(estimator_status)
    innov_flags = get_innovation_check_flags(estimator_status)
    gps_fail_flags = get_gps_check_fail_flags(estimator_status)
    return control_mode, innov_flags, gps_fail_flags


def get_control_mode_flags(estimator_status: dict) -> dict:
    """
    :param estimator_status:
    :return:
    """

    control_mode = dict()
    # extract control mode metadata from estimator_status.control_mode_flags
    # 0 - true if the filter tilt alignment is complete
    # 1 - true if the filter yaw alignment is complete
    # 2 - true if GPS measurements are being fused
    # 3 - true if optical flow measurements are being fused
    # 4 - true if a simple magnetic yaw heading is being fused
    # 5 - true if 3-axis magnetometer measurement are being fused
    # 6 - true if synthetic magnetic declination measurements are being fused
    # 7 - true when the vehicle is airborne
    # 8 - true when wind velocity is being estimated
    # 9 - true when baro height is being fused as a primary height reference
    # 10 - true when range finder height is being fused as a primary height reference
    # 11 - true when range finder height is being fused as a primary height reference
    # 12 - true when local position data from external vision is being fused
    # 13 - true when yaw data from external vision measurements is being fused
    # 14 - true when height data from external vision measurements is being fused
    # 15 - true when synthetic sideslip measurements are being fused
    # 16 - true true when the mag field does not match the expected strength
    # 17 - true true when the vehicle is operating as a fixed wing vehicle
    # 18 - true when the magnetometer has been declared faulty and is no longer being used
    # 19 - true true when airspeed measurements are being fused
    # 20 - true true when protection from ground effect induced static pressure rise is active
    # 21 - true when rng data wasn't ready for more than 10s and new rng values haven't changed enough
    # 22 - true when yaw (not ground course) data from a GPS receiver is being fused
    # 23 - true when the in-flight mag field alignment has been completed
    # 24 - true when local earth frame velocity data from external vision measurements are being fused
    # 25 - true when we are using a synthesized measurement for the magnetometer Z component
    control_mode['tilt_aligned'] = ((2 ** 0 & estimator_status['control_mode_flags']) > 0) * 1
    control_mode['yaw_aligned'] = ((2 ** 1 & estimator_status['control_mode_flags']) > 0) * 1
    control_mode['using_gps'] = ((2 ** 2 & estimator_status['control_mode_flags']) > 0) * 1
    control_mode['using_optflow'] = ((2 ** 3 & estimator_status['control_mode_flags']) > 0) * 1
    control_mode['using_magyaw'] = ((2 ** 4 & estimator_status['control_mode_flags']) > 0) * 1
    control_mode['using_mag3d'] = ((2 ** 5 & estimator_status['control_mode_flags']) > 0) * 1
    control_mode['using_magdecl'] = ((2 ** 6 & estimator_status['control_mode_flags']) > 0) * 1
    control_mode['airborne'] = ((2 ** 7 & estimator_status['control_mode_flags']) > 0) * 1
    control_mode['estimating_wind'] = ((2 ** 8 & estimator_status['control_mode_flags']) > 0) * 1
    control_mode['using_barohgt'] = ((2 ** 9 & estimator_status['control_mode_flags']) > 0) * 1
    control_mode['using_rnghgt'] = ((2 ** 10 & estimator_status['control_mode_flags']) > 0) * 1
    control_mode['using_gpshgt'] = ((2 ** 11 & estimator_status['control_mode_flags']) > 0) * 1
    control_mode['using_evpos'] = ((2 ** 12 & estimator_status['control_mode_flags']) > 0) * 1
    control_mode['using_evyaw'] = ((2 ** 13 & estimator_status['control_mode_flags']) > 0) * 1
    control_mode['using_evhgt'] = ((2 ** 14 & estimator_status['control_mode_flags']) > 0) * 1
    control_mode['fuse_beta'] = ((2 ** 15 & estimator_status['control_mode_flags']) > 0) * 1
    control_mode['mag_field_disturbed'] = ((2 ** 16 & estimator_status['control_mode_flags']) > 0) * 1
    control_mode['fixed_wing'] = ((2 ** 17 & estimator_status['control_mode_flags']) > 0) * 1
    control_mode['mag_fault'] = ((2 ** 18 & estimator_status['control_mode_flags']) > 0) * 1
    control_mode['fuse_aspd'] = ((2 ** 19 & estimator_status['control_mode_flags']) > 0) * 1
    control_mode['gnd_effect'] = ((2 ** 20 & estimator_status['control_mode_flags']) > 0) * 1
    control_mode['rng_stuck'] = ((2 ** 21 & estimator_status['control_mode_flags']) > 0) * 1
    control_mode['gps_yaw'] = ((2 ** 22 & estimator_status['control_mode_flags']) > 0) * 1
    control_mode['mag_aligned_in_flight'] = ((2 ** 23 & estimator_status['control_mode_flags']) > 0) * 1
    control_mode['ev_vel'] = ((2 ** 24 & estimator_status['control_mode_flags']) > 0) * 1
    control_mode['synthetic_mag_z'] = ((2 ** 25 & estimator_status['control_mode_flags']) > 0) * 1
    return control_mode

def get_innovation_check_flags(estimator_status: dict) -> dict:
    """
    :param estimator_status:
    :return:
    """

    innov_flags = dict()
    # innovation_check_flags summary
    # 0 - true if velocity observations have been rejected
    # 1 - true if horizontal position observations have been rejected
    # 2 - true if true if vertical position observations have been rejected
    # 3 - true if the X magnetometer observation has been rejected
    # 4 - true if the Y magnetometer observation has been rejected
    # 5 - true if the Z magnetometer observation has been rejected
    # 6 - true if the yaw observation has been rejected
    # 7 - true if the airspeed observation has been rejected
    # 8 - true if synthetic sideslip observation has been rejected
    # 9 - true if the height above ground observation has been rejected
    # 10 - true if the X optical flow observation has been rejected
    # 11 - true if the Y optical flow observation has been rejected
    innov_flags['vel_innov_fail'] = ((2 ** 0 & estimator_status['innovation_check_flags']) > 0) * 1
    innov_flags['posh_innov_fail'] = ((2 ** 1 & estimator_status['innovation_check_flags']) > 0) * 1
    innov_flags['posv_innov_fail'] = ((2 ** 2 & estimator_status['innovation_check_flags']) > 0) * 1
    innov_flags['magx_innov_fail'] = ((2 ** 3 & estimator_status['innovation_check_flags']) > 0) * 1
    innov_flags['magy_innov_fail'] = ((2 ** 4 & estimator_status['innovation_check_flags']) > 0) * 1
    innov_flags['magz_innov_fail'] = ((2 ** 5 & estimator_status['innovation_check_flags']) > 0) * 1
    innov_flags['yaw_innov_fail'] = ((2 ** 6 & estimator_status['innovation_check_flags']) > 0) * 1
    innov_flags['tas_innov_fail'] = ((2 ** 7 & estimator_status['innovation_check_flags']) > 0) * 1
    innov_flags['sli_innov_fail'] = ((2 ** 8 & estimator_status['innovation_check_flags']) > 0) * 1
    innov_flags['hagl_innov_fail'] = ((2 ** 9 & estimator_status['innovation_check_flags']) > 0) * 1
    innov_flags['ofx_innov_fail'] = ((2 ** 10 & estimator_status['innovation_check_flags']) > 0) * 1
    innov_flags['ofy_innov_fail'] = ((2 ** 11 & estimator_status['innovation_check_flags']) > 0) * 1
    return innov_flags


def get_gps_check_fail_flags(estimator_status: dict) -> dict:
    """
    :param estimator_status:
    :return:
    """
    gps_fail_flags = dict()

    # 0 : insufficient fix type (no 3D solution)
    # 1 : minimum required sat count fail
    # 2 : maximum allowed PDOP fail
    # 3 : maximum allowed horizontal position error fail
    # 4 : maximum allowed vertical position error fail
    # 5 : maximum allowed speed error fail
    # 6 : maximum allowed horizontal position drift fail
    # 7 : maximum allowed vertical position drift fail
    # 8 : maximum allowed horizontal speed fail
    # 9 : maximum allowed vertical velocity discrepancy fail
    gps_fail_flags['gfix_fail'] = ((2 ** 0 & estimator_status['gps_check_fail_flags']) > 0) * 1
    gps_fail_flags['nsat_fail'] = ((2 ** 1 & estimator_status['gps_check_fail_flags']) > 0) * 1
    gps_fail_flags['pdop_fail'] = ((2 ** 2 & estimator_status['gps_check_fail_flags']) > 0) * 1
    gps_fail_flags['herr_fail'] = ((2 ** 3 & estimator_status['gps_check_fail_flags']) > 0) * 1
    gps_fail_flags['verr_fail'] = ((2 ** 4 & estimator_status['gps_check_fail_flags']) > 0) * 1
    gps_fail_flags['serr_fail'] = ((2 ** 5 & estimator_status['gps_check_fail_flags']) > 0) * 1
    gps_fail_flags['hdrift_fail'] = ((2 ** 6 & estimator_status['gps_check_fail_flags']) > 0) * 1
    gps_fail_flags['vdrift_fail'] = ((2 ** 7 & estimator_status['gps_check_fail_flags']) > 0) * 1
    gps_fail_flags['hspd_fail'] = ((2 ** 8 & estimator_status['gps_check_fail_flags']) > 0) * 1
    gps_fail_flags['veld_diff_fail'] = ((2 ** 9 & estimator_status['gps_check_fail_flags']) > 0) * 1
    return gps_fail_flags


def magnetic_field_estimates_from_states(estimator_states: dict) -> Tuple[float, float, float]:
    """

    :param estimator_states:
    :return:
    """
    rad2deg = 57.2958
    field_strength = np.sqrt(
        estimator_states['states[16]'] ** 2 + estimator_states['states[17]'] ** 2 +
        estimator_states['states[18]'] ** 2)
    declination = rad2deg * np.arctan2(estimator_states['states[17]'],
                                       estimator_states['states[16]'])
    inclination = rad2deg * np.arcsin(
        estimator_states['states[18]'] / np.maximum(field_strength, np.finfo(np.float32).eps))
    return declination, field_strength, inclination
