#!/usr/bin/env python3
"""
Script to parse uORB message format to ROS msg format
Adapted from https://github.com/eProsima/px4_to_ros/blob/master/px4_to_ros2_PoC/px4_msgs/scripts/copy_and_rename.py
"""

import os
import re
import sys
from shutil import copyfile

__author__ = 'PX4 Development Team'
__copyright__ = \
    '''
     '
     '   Copyright (c) 2018 PX4 Development Team. All rights reserved.
     '
     ' Redistribution and use in source and binary forms, or without
     ' modification, permitted provided that the following conditions
     ' are met:
     '
     ' 1. Redistributions of source code must retain the above copyright
     '    notice, list of conditions and the following disclaimer.
     ' 2. Redistributions in binary form must reproduce the above copyright
     '    notice, list of conditions and the following disclaimer in
     '    the documentation and/or other materials provided with the
     '    distribution.
     ' 3. Neither the name PX4 nor the names of its contributors may be
     '    used to endorse or promote products derived from self software
     '    without specific prior written permission.
     '
     ' THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
     ' "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, NOT
     ' LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
     ' FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
     ' COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
     ' INCIDENTAL, SPECIAL, EXEMPLARY, CONSEQUENTIAL DAMAGES (INCLUDING,
     ' BUT NOT LIMITED TO, OF SUBSTITUTE GOODS OR SERVICES; LOSS
     ' OF USE, DATA, PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
     ' AND ON ANY THEORY OF LIABILITY, IN CONTRACT, STRICT
     ' LIABILITY, TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
     ' ANY WAY OUT OF THE USE OF THIS SOFTWARE, IF ADVISED OF THE
     ' POSSIBILITY OF SUCH DAMAGE.
     '
    '''
__credits__ = ['Nuno Marques <nuno.marques@dronesolution.io>']
__license__ = 'BSD-3-Clause'
__version__ = '0.1.0'
__maintainer__ = 'Nuno Marques'
__email__ = 'nuno.marques@dronesolution.io'
__status__ = 'Development'

input_dir = sys.argv[1]
output_dir = sys.argv[2]

if not os.path.exists(os.path.abspath(output_dir)):
    os.mkdir(os.path.abspath(output_dir))
else:
    ros_msg_dir = os.path.abspath(output_dir)
    msg_files = os.listdir(ros_msg_dir)
    for msg in msg_files:
        if msg.endswith(".msg"):
            os.remove(os.path.join(ros_msg_dir, msg))

msg_list = list()

for filename in os.listdir(input_dir):
    if '.msg' in filename:
        msg_list.append(filename.rstrip('.msg'))
        input_file = input_dir + filename

        output_file = output_dir + \
            filename.partition(".")[0].title().replace('_', '') + ".msg"
        copyfile(input_file, output_file)

for filename in os.listdir(output_dir):
    if '.msg' in filename:
        input_file = output_dir + filename

        fileUpdated = False

        with open(input_file, 'r') as f:
            lines = f.readlines()
            newlines = []
            alias_msgs = []
            alias_msg_files = []

            for line in lines:
                for msg_type in msg_list:
                    if ('px4/' + msg_type + ' ') in line:
                        fileUpdated = True
                        line = line.replace(('px4/' + msg_type),
                                            msg_type.partition(".")[0].title().replace('_', ''))
                    if re.findall('^' + msg_type + '[\s\[]', line.partition('#')[0]):
                        fileUpdated = True
                        line = line.replace(msg_type,
                                            msg_type.partition(".")[0].title().replace('_', ''))
                    if '# TOPICS' in line:
                        fileUpdated = True
                        alias_msgs += line.split()
                        alias_msgs.remove('#')
                        alias_msgs.remove('TOPICS')
                        line = line.replace(line, '')
                newlines.append(line)

        for msg_file in alias_msgs:
            with open(output_dir + msg_file.partition(".")[0].title().replace('_', '') + ".msg", 'w+') as f:
                for line in newlines:
                    f.write(line)

        if fileUpdated:
            with open(input_file, 'w+') as f:
                for line in newlines:
                    f.write(line)
