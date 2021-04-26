/*
 * uorb_converter.h
 *
 *  Created on: Apr 10, 2020
 *      Author: hovergames
 */

#ifndef SRC_DRIVERS_UAVCANNODE_V1_UORB_CONVERTER_H_
#define SRC_DRIVERS_UAVCANNODE_V1_UORB_CONVERTER_H_


/* uORB */
#include <px4_platform_common/posix.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_gps.h>

/* canard */
#include <canard.h>
#include <canard_dsdl.h>

/* monotonic timestamp */
#include "libcancl/time.h"


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

void uorbConverterInit(CanardInstance *ins, int16_t *raw_uorb_port_id, int16_t *fix_port_id, int16_t *aux_port_id);

void uorbProcessSub(int timeout_msec);


#endif /* SRC_DRIVERS_UAVCANNODE_V1_UORB_CONVERTER_H_ */
