#!/usr/bin/env python
############################################################################
#
#   Copyright (c) 2017-2020 PX4 Development Team. All rights reserved.
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
############################################################################

"""
    Basic central executable for PX4 maintenance.
    Used by PX4 Homebrew formulae - https://github.com/PX4/homebrew-px4.
"""

# For python2.7 compatibility
from __future__ import print_function

import argparse
import array
import base64
import binascii
import json
import os
import serial
import socket
import struct
import subprocess
import sys
import time
import zlib

from sys import platform as _platform


def get_version():
    """
        Get PX4 Firmware latest Git tag.
    """
    px4_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    if os.path.isdir(os.path.join(px4_dir, '.git')):
        # If inside a clone PX4 Firmware repository, get version from "git describe"
        cmd = 'git describe --exclude ext/* --abbrev=0 --tags'
        try:
            version = subprocess.check_output(
                cmd, cwd=px4_dir, shell=True).decode().strip()
        except subprocess.CalledProcessError:
            print('Unable to get version number from git tags')
            exit(1)
    else:
        # Else, get it from remote repo tags (requires network access)
        cmd = "git ls-remote --tags git://github.com/PX4/Firmware.git | cut -d/ -f3- | sort -n -t. -k1,1 -k2,2 -k3,3 | awk '/^v[^{]*$/{version=$1}END{print version}'"

        try:
            version = subprocess.check_output(
                cmd, cwd=px4_dir, shell=True).decode().strip()
        except subprocess.CalledProcessError:
            print('Unable to get version number from git remote tags')
            exit(1)

    return version


# Detect python version
if sys.version_info[0] < 3:
    runningPython3 = False
else:
    runningPython3 = True


def main():
    print("PX4 release", get_version())


if __name__ == '__main__':
    main()
