#!/usr/bin/env python2
#***************************************************************************
#
#   Copyright (c) 2015-2016 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
#***************************************************************************/

#
# @author Andreas Antener <andreas@uaventure.com>
#

# The shebang of this file is currently Python2 because some
# dependencies such as pymavlink don't play well with Python3 yet.
from __future__ import division

PKG = 'px4'

import rospy
import glob
import json
import math
import os
from px4tools import ulog
import sys
from mavros import mavlink
from mavros_msgs.msg import Mavlink, Waypoint, WaypointReached
from mavros_test_common import MavrosTestCommon
from pymavlink import mavutil
from six.moves import xrange
from threading import Thread


def get_last_log():
    try:
        log_path = os.environ['PX4_LOG_DIR']
    except KeyError:
        try:
            log_path = os.path.join(os.environ['ROS_HOME'], 'log')
        except KeyError:
            log_path = os.path.join(os.environ['HOME'], '.ros/log')
    last_log_dir = sorted(glob.glob(os.path.join(log_path, '*')))[-1]
    last_log = sorted(glob.glob(os.path.join(last_log_dir, '*.ulg')))[-1]
    return last_log


def read_mission(mission_filename):
    wps = []
    with open(mission_filename, 'r') as f:
        for waypoint in read_plan_file(f):
            wps.append(waypoint)
            rospy.logdebug(waypoint)

    # set first item to current
    if wps:
        wps[0].is_current = True

    return wps


def read_plan_file(f):
    d = json.load(f)
    if 'mission' in d:
        d = d['mission']

    if 'items' in d:
        for wp in d['items']:
            yield Waypoint(
                is_current=False,
                frame=int(wp['frame']),
                command=int(wp['command']),
                param1=float('nan'
                             if wp['params'][0] is None else wp['params'][0]),
                param2=float('nan'
                             if wp['params'][1] is None else wp['params'][1]),
                param3=float('nan'
                             if wp['params'][2] is None else wp['params'][2]),
                param4=float('nan'
                             if wp['params'][3] is None else wp['params'][3]),
                x_lat=float(wp['params'][4]),
                y_long=float(wp['params'][5]),
                z_alt=float(wp['params'][6]),
                autocontinue=bool(wp['autoContinue']))
    else:
        raise IOError("no mission items")


class MavrosMissionTest(MavrosTestCommon):
    """
    Run a mission
    """

    def setUp(self):
        super(self.__class__, self).setUp()

        self.mission_item_reached = -1  # first mission item is 0
        self.mission_name = ""

        self.mavlink_pub = rospy.Publisher('mavlink/to', Mavlink, queue_size=1)
        self.mission_item_reached_sub = rospy.Subscriber(
            'mavros/mission/reached', WaypointReached,
            self.mission_item_reached_callback)

        # need to simulate heartbeat to prevent datalink loss detection
        self.hb_mav_msg = mavutil.mavlink.MAVLink_heartbeat_message(
            mavutil.mavlink.MAV_TYPE_GCS, 0, 0, 0, 0, 0)
        self.hb_mav_msg.pack(mavutil.mavlink.MAVLink('', 2, 1))
        self.hb_ros_msg = mavlink.convert_to_rosmsg(self.hb_mav_msg)
        self.hb_thread = Thread(target=self.send_heartbeat, args=())
        self.hb_thread.daemon = True
        self.hb_thread.start()

    def tearDown(self):
        super(MavrosMissionTest, self).tearDown()

    #
    # Helper methods
    #
    def send_heartbeat(self):
        rate = rospy.Rate(2)  # Hz
        while not rospy.is_shutdown():
            self.mavlink_pub.publish(self.hb_ros_msg)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def mission_item_reached_callback(self, data):
        if self.mission_item_reached != data.wp_seq:
            rospy.loginfo("mission item reached: {0}".format(data.wp_seq))
            self.mission_item_reached = data.wp_seq

    def distance_to_wp(self, lat, lon, alt):
        """alt(amsl): meters"""
        R = 6371000  # metres
        rlat1 = math.radians(lat)
        rlat2 = math.radians(self.global_position.latitude)

        rlat_d = math.radians(self.global_position.latitude - lat)
        rlon_d = math.radians(self.global_position.longitude - lon)

        a = (math.sin(rlat_d / 2) * math.sin(rlat_d / 2) + math.cos(rlat1) *
             math.cos(rlat2) * math.sin(rlon_d / 2) * math.sin(rlon_d / 2))
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        d = R * c
        alt_d = abs(alt - self.altitude.amsl)

        rospy.logdebug("d: {0}, alt_d: {1}".format(d, alt_d))
        return d, alt_d

    def reach_position(self, lat, lon, alt, timeout, index):
        """alt(amsl): meters, timeout(int): seconds"""
        rospy.loginfo(
            "trying to reach waypoint | lat: {0:.9f}, lon: {1:.9f}, alt: {2:.2f}, index: {3}".
            format(lat, lon, alt, index))
        best_pos_xy_d = None
        best_pos_z_d = None
        reached = False
        mission_length = len(self.mission_wp.waypoints)

        # does it reach the position in 'timeout' seconds?
        loop_freq = 2  # Hz
        rate = rospy.Rate(loop_freq)
        for i in xrange(timeout * loop_freq):
            pos_xy_d, pos_z_d = self.distance_to_wp(lat, lon, alt)

            # remember best distances
            if not best_pos_xy_d or best_pos_xy_d > pos_xy_d:
                best_pos_xy_d = pos_xy_d
            if not best_pos_z_d or best_pos_z_d > pos_z_d:
                best_pos_z_d = pos_z_d

            # FCU advanced to the next mission item, or finished mission
            reached = (
                # advanced to next wp
                (index < self.mission_wp.current_seq)
                # end of mission
                or (index == (mission_length - 1) and
                    self.mission_item_reached == index))

            if reached:
                rospy.loginfo(
                    "position reached | pos_xy_d: {0:.2f}, pos_z_d: {1:.2f}, index: {2} | seconds: {3} of {4}".
                    format(pos_xy_d, pos_z_d, index, i / loop_freq, timeout))
                break
            elif i == 0 or ((i / loop_freq) % 10) == 0:
                # log distance first iteration and every 10 sec
                rospy.loginfo(
                    "current distance to waypoint | pos_xy_d: {0:.2f}, pos_z_d: {1:.2f}, index: {2}".
                    format(pos_xy_d, pos_z_d, index))

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        self.assertTrue(
            reached,
            "position not reached | lat: {0:.9f}, lon: {1:.9f}, alt: {2:.2f}, current pos_xy_d: {3:.2f}, current pos_z_d: {4:.2f}, best pos_xy_d: {5:.2f}, best pos_z_d: {6:.2f}, index: {7} | timeout(seconds): {8}".
            format(lat, lon, alt, pos_xy_d, pos_z_d, best_pos_xy_d,
                   best_pos_z_d, index, timeout))

    #
    # Test method
    #
    def test_mission(self):
        """Test mission"""
        if len(sys.argv) < 2:
            self.fail("usage: mission_test.py mission_file")
            return

        self.mission_name = sys.argv[1]
        mission_file = os.path.dirname(
            os.path.realpath(__file__)) + "/missions/" + sys.argv[1]

        rospy.loginfo("reading mission {0}".format(mission_file))
        try:
            wps = read_mission(mission_file)
        except IOError as e:
            self.fail(e)

        # make sure the simulation is ready to start the mission
        self.wait_for_topics(60)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   10, -1)
        self.wait_for_mav_type(10)

        # push waypoints to FCU and start mission
        self.send_wps(wps, 30)
        self.log_topic_vars()
        self.set_mode("AUTO.MISSION", 5)
        self.set_arm(True, 5)

        rospy.loginfo("run mission {0}".format(self.mission_name))
        for index, waypoint in enumerate(wps):
            # only check position for waypoints where this makes sense
            if (waypoint.frame == Waypoint.FRAME_GLOBAL_REL_ALT or
                    waypoint.frame == Waypoint.FRAME_GLOBAL):
                alt = waypoint.z_alt
                if waypoint.frame == Waypoint.FRAME_GLOBAL_REL_ALT:
                    alt += self.altitude.amsl - self.altitude.relative

                self.reach_position(waypoint.x_lat, waypoint.y_long, alt, 60,
                                    index)

            # check if VTOL transition happens if applicable
            if (waypoint.command == mavutil.mavlink.MAV_CMD_NAV_VTOL_TAKEOFF or
                    waypoint.command == mavutil.mavlink.MAV_CMD_NAV_VTOL_LAND
                    or waypoint.command ==
                    mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION):
                transition = waypoint.param1  # used by MAV_CMD_DO_VTOL_TRANSITION
                if waypoint.command == mavutil.mavlink.MAV_CMD_NAV_VTOL_TAKEOFF:  # VTOL takeoff implies transition to FW
                    transition = mavutil.mavlink.MAV_VTOL_STATE_FW
                if waypoint.command == mavutil.mavlink.MAV_CMD_NAV_VTOL_LAND:  # VTOL land implies transition to MC
                    transition = mavutil.mavlink.MAV_VTOL_STATE_MC

                self.wait_for_vtol_state(transition, 60, index)

            # after reaching position, wait for landing detection if applicable
            if (waypoint.command == mavutil.mavlink.MAV_CMD_NAV_VTOL_LAND or
                    waypoint.command == mavutil.mavlink.MAV_CMD_NAV_LAND):
                self.wait_for_landed_state(
                    mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 120, index)

        self.set_arm(False, 5)
        self.clear_wps(5)

        rospy.loginfo("mission done, calculating performance metrics")
        last_log = get_last_log()
        rospy.loginfo("log file {0}".format(last_log))
        data = ulog.read_ulog(last_log).concat(dt=0.1)
        data = ulog.compute_data(data)
        res = ulog.estimator_analysis(data, False)

        # enforce performance
        self.assertTrue(abs(res['roll_error_mean']) < 5.0, str(res))
        self.assertTrue(abs(res['pitch_error_mean']) < 5.0, str(res))
        self.assertTrue(abs(res['yaw_error_mean']) < 5.0, str(res))
        self.assertTrue(res['roll_error_std'] < 5.0, str(res))
        self.assertTrue(res['pitch_error_std'] < 5.0, str(res))
        self.assertTrue(res['yaw_error_std'] < 5.0, str(res))


if __name__ == '__main__':
    import rostest
    rospy.init_node('test_node', anonymous=True)

    name = "mavros_mission_test"
    if len(sys.argv) > 1:
        name += "-%s" % sys.argv[1]
    rostest.rosrun(PKG, name, MavrosMissionTest)
