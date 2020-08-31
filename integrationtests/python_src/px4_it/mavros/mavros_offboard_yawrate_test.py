#!/usr/bin/env python2
#***************************************************************************
#
#   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
# @author Pedro Roque <padr@kth.se>
#

from __future__ import division

PKG = 'px4'

import rospy
from geometry_msgs.msg import Quaternion, Vector3
from mavros_msgs.msg import AttitudeTarget
from mavros_test_common import MavrosTestCommon
from pymavlink import mavutil
from six.moves import xrange
from std_msgs.msg import Header
from threading import Thread
from tf.transformations import quaternion_from_euler


class MavrosOffboardYawrateTest(MavrosTestCommon):
    """
    Tests flying in offboard control by sending a Roll Pitch Yawrate Thrust (RPYrT)
    as attitude setpoint.

    For the test to be successful it needs to achieve a desired yawrate and height.
    """

    def setUp(self):
        super(MavrosOffboardYawrateTest, self).setUp()

        self.att = AttitudeTarget()

        self.att_setpoint_pub = rospy.Publisher(
            'mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)

        # send setpoints in seperate thread to better prevent failsafe
        self.att_thread = Thread(target=self.send_att, args=())
        self.att_thread.daemon = True
        self.att_thread.start()

        # desired yawrate target
        self.des_yawrate = 0.1
        self.yawrate_tol = 0.02

    def tearDown(self):
        super(MavrosOffboardYawrateTest, self).tearDown()

    #
    # Helper methods
    #
    def send_att(self):
        rate = rospy.Rate(10)  # Hz
        self.att.body_rate = Vector3()
        self.att.header = Header()
        self.att.header.frame_id = "base_footprint"
        self.att.orientation = self.local_position.pose.orientation

        self.att.body_rate.x = 0
        self.att.body_rate.y = 0
        self.att.body_rate.z = self.des_yawrate

        self.att.thrust = 0.59

        self.att.type_mask = 3 # ignore roll and pitch rate

        while not rospy.is_shutdown():
            self.att.header.stamp = rospy.Time.now()
            self.att_setpoint_pub.publish(self.att)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    #
    # Test method
    #
    def test_attctl(self):
        """Test offboard yawrate control"""

        # boundary to cross
        # Stay leveled, go up, and test yawrate
        boundary_x = 5
        boundary_y = 5
        boundary_z = 10


        # make sure the simulation is ready to start the mission
        self.wait_for_topics(60)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   10, -1)

        self.log_topic_vars()
        self.set_arm(True, 5)
        self.set_mode("OFFBOARD", 5)
        rospy.loginfo("run mission")
        rospy.loginfo("attempting to cross boundary | z: {2} , stay within x: {0}  y: {1} \n   and achieve {3} yawrate".
                      format(boundary_x, boundary_y, boundary_z, self.des_yawrate))

	    # does it cross expected boundaries in 'timeout' seconds?
        timeout = 90  # (int) seconds
        loop_freq = 2  # Hz
        rate = rospy.Rate(loop_freq)
        crossed = False
        for i in xrange(timeout * loop_freq):
            if (self.local_position.pose.position.x < boundary_x and
	    	    self.local_position.pose.position.x > -boundary_x and
                    self.local_position.pose.position.y < boundary_y and
		    self.local_position.pose.position.y > -boundary_y and
                    self.local_position.pose.position.z > boundary_z and
		    abs(self.imu_data.angular_velocity.z - self.des_yawrate) < self.yawrate_tol):
                rospy.loginfo("Test successful. Final altitude and yawrate achieved")
                crossed = True
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        self.assertTrue(crossed, (
            "took too long to finish test | current position x: {0:.2f}, y: {1:.2f}, z: {2:.2f} \n " \
	    "                             | current att qx: {3:.2f}, qy: {4:.2f}, qz: {5:.2f} qw: {6:.2f}, yr: {7:.2f}| timeout(seconds): {8}".
            format(self.local_position.pose.position.x,
                   self.local_position.pose.position.y,
                   self.local_position.pose.position.z,
		   self.imu_data.orientation.x,
		   self.imu_data.orientation.y,
		   self.imu_data.orientation.z,
		   self.imu_data.orientation.w,
		   self.imu_data.angular_velocity.z,
		   timeout)))

        self.set_mode("AUTO.LAND", 5)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   90, 0)
        self.set_arm(False, 5)


if __name__ == '__main__':
    import rostest
    rospy.init_node('test_node', anonymous=True)

    rostest.rosrun(PKG, 'mavros_offboard_yawrate_test',
                   MavrosOffboardYawrateTest)
