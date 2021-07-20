/* This file would be automatically generated like px4_parameters.h */

#pragma once

#include <parameters/param.h>

#define N_UAVCAN_PARAMS 5  // temporary macro to define number of parameters

/**
 * Parameter information
 */
struct uavcan_param_info_s {
	const char *name;				/** The UAVCAN parameter name */
	const char *px4_param_name;		/** Name of PX4 parameter that stores the UAVCAN parameter data */
};

/**
 * Static parameter information
 */
struct uavcan_parameters_t {
	const struct uavcan_param_info_s bus_bitrate;
	const struct uavcan_param_info_s node_id;
	const struct uavcan_param_info_s uavcan_rawairdata_pub_period;
	const struct uavcan_param_info_s uavcan_staticpressure_pub_period;
	const struct uavcan_param_info_s uavcan_magneticfieldstrength2_pub_period;
	const unsigned int param_count;
};

extern const struct uavcan_parameters_t uavcan_parameters;
extern param_t uavcan_param_map[N_UAVCAN_PARAMS];
