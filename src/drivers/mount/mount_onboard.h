/****************************************************************************
*
*   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
* 3. Neither the name PX4 nor the names of its contributors may be
*    used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
****************************************************************************/

/**
 * @file mount_onboard
 * @author Leon Müller (thedevleon)
 *
 */

#ifndef _MOUNT_ONBOARD_H
#define _MOUNT_ONBOARD_H

#include <sys/types.h>
#include <stdbool.h>

// Public functions
bool mount_onboard_init(void);
void mount_onboard_deinit(void);
void mount_onboard_configure(int new_mount_mode, bool new_stab_roll, bool new_stab_pitch, bool new_stab_yaw);
void mount_onboard_set_location(int new_mount_mode, double global_lat, double global_lon, float global_alt, double lat,
				double lon, float alt);
void mount_onboard_set_manual(int new_mount_mode, float new_pitch, float new_roll, float new_yaw);
void mount_onboard_point(void);
void mount_onboard_update_topics(void);
float mount_onboard_calculate_pitch(double global_lat, double global_lon, float global_alt, double lat, double lon,
				    float alt);

#endif /* _MOUNT_ONBOARD_H */
