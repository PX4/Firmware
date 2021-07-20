/* This file would be automatically generated like px4_parameters.c */

#include "uavcan_parameters.h"

const
#ifndef __PX4_DARWIN
__attribute__((used, section("__param")))
#endif

struct uavcan_parameters_t uavcan_parameters = {
	{
		"uavcan.bit_rate",
		"CANNODE_BITRATE"
	},

	{
		"uavcan.node_id",
		"CANNODE_NODE_ID"
	},

	{
		"uavcan.pubp-uavcan.equipment.ahrs.MagneticFieldStrength2",
		"UAVCAN_MAG_T"
	},

	{
		"uavcan.pubp-uavcan.equipment.air_data.RawAirData",
		"UAVCAN_ARSP_T"
	},

	{
		"uavcan.pubp-uavcan.equipment.air_data.StaticPressure",
		"UAVCAN_BARO_T"
	},

	N_UAVCAN_PARAMS
};

/**
 * Run-time parameter mapping from uavcan indices to px4 parameter handles
 * This could be defined at compile-time and combined into uavcan_parameters by parsing px4_parameters.c
 */
param_t uavcan_param_map[N_UAVCAN_PARAMS];
