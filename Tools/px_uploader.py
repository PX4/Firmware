#!/usr/bin/env python3
############################################################################
#
#   Copyright (c) 2012-2017 PX4 Development Team. All rights reserved.
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

#
# Serial firmware uploader for the PX4FMU bootloader
#
# The PX4 firmware file is a JSON-encoded Python object, containing
# metadata fields and a zlib-compressed base64-encoded firmware image.
#
# The uploader uses the following fields from the firmware file:
#
# image
#       The firmware that will be uploaded.
# image_size
#       The size of the firmware in bytes.
# board_id
#       The board for which the firmware is intended.
# board_revision
#       Currently only used for informational purposes.
#

# for python2.7 compatibility
from __future__ import print_function

import sys
import argparse
import binascii
import serial
import socket
import struct
import json
import zlib
import base64
import time
import array
import os

from sys import platform as _platform

# Detect python version
if sys.version_info[0] < 3:
    runningPython3 = False
else:
    runningPython3 = True

class FirmwareNotSuitableException(Exception):
    def __init__(self, message):
        super(FirmwareNotSuitableException, self).__init__(message)

class firmware(object):
    '''Loads a firmware file'''

    desc = {}
    image = bytes()
    crctab = array.array('I', [
        0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f, 0xe963a535, 0x9e6495a3,
        0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988, 0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91,
        0x1db71064, 0x6ab020f2, 0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
        0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9, 0xfa0f3d63, 0x8d080df5,
        0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172, 0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b,
        0x35b5a8fa, 0x42b2986c, 0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
        0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423, 0xcfba9599, 0xb8bda50f,
        0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924, 0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d,
        0x76dc4190, 0x01db7106, 0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
        0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d, 0x91646c97, 0xe6635c01,
        0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e, 0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457,
        0x65b0d9c6, 0x12b7e950, 0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
        0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7, 0xa4d1c46d, 0xd3d6f4fb,
        0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0, 0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9,
        0x5005713c, 0x270241aa, 0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
        0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81, 0xb7bd5c3b, 0xc0ba6cad,
        0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a, 0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683,
        0xe3630b12, 0x94643b84, 0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
        0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb, 0x196c3671, 0x6e6b06e7,
        0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc, 0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5,
        0xd6d6a3e8, 0xa1d1937e, 0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
        0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55, 0x316e8eef, 0x4669be79,
        0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236, 0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f,
        0xc5ba3bbe, 0xb2bd0b28, 0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
        0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f, 0x72076785, 0x05005713,
        0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38, 0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21,
        0x86d3d2d4, 0xf1d4e242, 0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
        0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69, 0x616bffd3, 0x166ccf45,
        0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2, 0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db,
        0xaed16a4a, 0xd9d65adc, 0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
        0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693, 0x54de5729, 0x23d967bf,
        0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94, 0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d])
    crcpad = bytearray(b'\xff\xff\xff\xff')

    def __init__(self, path):

        # read the file
        f = open(path, "r")
        self.desc = json.load(f)
        f.close()

        self.image = bytearray(zlib.decompress(base64.b64decode(self.desc['image'])))

        # pad image to 4-byte length
        while ((len(self.image) % 4) != 0):
            self.image.extend(b'\xff')

    def property(self, propname):
        return self.desc[propname]

    def __crc32(self, bytes, state):
        for byte in bytes:
            index = (state ^ byte) & 0xff
            state = self.crctab[index] ^ (state >> 8)
        return state

    def crc(self, padlen):
        state = self.__crc32(self.image, int(0))
        for i in range(len(self.image), (padlen - 1), 4):
            state = self.__crc32(self.crcpad, state)
        return state


class uploader(object):
    '''Uploads a firmware file to the PX FMU bootloader'''

    # protocol bytes
    INSYNC          = b'\x12'
    EOC             = b'\x20'

    # reply bytes
    OK              = b'\x10'
    FAILED          = b'\x11'
    INVALID         = b'\x13'     # rev3+
    BAD_SILICON_REV = b'\x14'     # rev5+

    # command bytes
    NOP             = b'\x00'     # guaranteed to be discarded by the bootloader
    GET_SYNC        = b'\x21'
    GET_DEVICE      = b'\x22'
    CHIP_ERASE      = b'\x23'
    CHIP_VERIFY     = b'\x24'     # rev2 only
    PROG_MULTI      = b'\x27'
    READ_MULTI      = b'\x28'     # rev2 only
    GET_CRC         = b'\x29'     # rev3+
    GET_OTP         = b'\x2a'     # rev4+  , get a word from OTP area
    GET_SN          = b'\x2b'     # rev4+  , get a word from SN area
    GET_CHIP        = b'\x2c'     # rev5+  , get chip version
    SET_BOOT_DELAY  = b'\x2d'     # rev5+  , set boot delay
    GET_CHIP_DES    = b'\x2e'     # rev5+  , get chip description in ASCII
    MAX_DES_LENGTH  = 20

    REBOOT          = b'\x30'

    INFO_BL_REV     = b'\x01'        # bootloader protocol revision
    BL_REV_MIN      = 2              # minimum supported bootloader protocol
    BL_REV_MAX      = 5              # maximum supported bootloader protocol
    INFO_BOARD_ID   = b'\x02'        # board type
    INFO_BOARD_REV  = b'\x03'        # board revision
    INFO_FLASH_SIZE = b'\x04'        # max firmware size in bytes

    PROG_MULTI_MAX  = 252            # protocol max is 255, must be multiple of 4
    READ_MULTI_MAX  = 252            # protocol max is 255

    NSH_INIT        = bytearray(b'\x0d\x0d\x0d')
    NSH_REBOOT_BL   = b"reboot -b\n"
    NSH_REBOOT      = b"reboot\n"
    MAVLINK_REBOOT_ID1 = bytearray(b'\xfe\x21\x72\xff\x00\x4c\x00\x00\x40\x40\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\xf6\x00\x01\x00\x00\x53\x6b')
    MAVLINK_REBOOT_ID0 = bytearray(b'\xfe\x21\x45\xff\x00\x4c\x00\x00\x40\x40\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\xf6\x00\x00\x00\x00\xcc\x37')

    MAX_FLASH_PRGRAM_TIME  = 0.001  # Time on an F7 to send SYNC, RESULT from last data in multi RXed

    def __init__(self, portname, baudrate_bootloader, baudrate_flightstack):
        # Open the port, keep the default timeout short so we can poll quickly.
        # On some systems writes can suddenly get stuck without having a
        # write_timeout > 0 set.
        # chartime 8n1 * bit rate is us
        self.chartime = 10 * (1.0 / baudrate_bootloader)

        # we use a window approche to SYNC,<result> gathring
        self.window = 0
        self.window_max = 256
        self.window_per = 2  # Sync,<result>
        self.ackWindowedMode = False  # Assume Non Widowed mode for all USB CDC
        self.port = serial.Serial(portname, baudrate_bootloader, timeout=0.5, write_timeout=0)
        self.otp = b''
        self.sn = b''
        self.baudrate_bootloader = baudrate_bootloader
        self.baudrate_flightstack = baudrate_flightstack
        self.baudrate_flightstack_idx = -1

    def close(self):
        if self.port is not None:
            self.port.close()

    def open(self):
        # upload timeout
        timeout = time.time() + 0.2

        # attempt to open the port while it exists and until timeout occurs
        while self.port is not None:
            portopen = True
            try:
                portopen = self.port.is_open
            except AttributeError:
                portopen = self.port.isOpen()

            if not portopen and time.time() < timeout:
                try:
                    self.port.open()
                except OSError:
                    # wait for the port to be ready
                    time.sleep(0.04)
                except serial.SerialException:
                    # if open fails, try again later
                    time.sleep(0.04)
            else:
                break

    # debugging code
    def __probe(self, state):
        # self.port.setRTS(state)
        return

    def __send(self, c):
        # print("send " + binascii.hexlify(c))
        self.port.write(c)

    def __recv(self, count=1):
        c = self.port.read(count)
        if len(c) < 1:
            raise RuntimeError("timeout waiting for data (%u bytes)" % count)
        # print("recv " + binascii.hexlify(c))
        return c

    def __recv_int(self):
        raw = self.__recv(4)
        val = struct.unpack("<I", raw)
        return val[0]

    def __getSync(self, doFlush=True):
        if (doFlush):
            self.port.flush()
        c = bytes(self.__recv())
        if c != self.INSYNC:
            raise RuntimeError("unexpected %s instead of INSYNC" % c)
        c = self.__recv()
        if c == self.INVALID:
            raise RuntimeError("bootloader reports INVALID OPERATION")
        if c == self.FAILED:
            raise RuntimeError("bootloader reports OPERATION FAILED")
        if c != self.OK:
            raise RuntimeError("unexpected response 0x%x instead of OK" % ord(c))

    # The control flow for reciving Sync is on the order of 16 Ms per Sync
    # This will validate all the SYNC,<results> for a window of programing
    # in about 13.81 Ms for 256 blocks written
    def __ackSyncWindow(self, count):
        if (count > 0):
            data =  bytearray(bytes(self.__recv(count)))
            if (len(data) != count):
                raise RuntimeError("Ack Window %i not %i " % (len(data), count))
            for i in range(0, len(data), 2):
                if bytes([data[i]]) != self.INSYNC:
                    raise RuntimeError("unexpected %s instead of INSYNC" % data[i])
                if bytes([data[i+1]]) == self.INVALID:
                    raise RuntimeError("bootloader reports INVALID OPERATION")
                if bytes([data[i+1]]) == self.FAILED:
                    raise RuntimeError("bootloader reports OPERATION FAILED")
                if bytes([data[i+1]]) != self.OK:
                    raise RuntimeError("unexpected response 0x%x instead of OK" % ord(data[i+1]))

    # attempt to get back into sync with the bootloader
    def __sync(self):
        # send a stream of ignored bytes longer than the longest possible conversation
        # that we might still have in progress
        # self.__send(uploader.NOP * (uploader.PROG_MULTI_MAX + 2))
        self.port.flushInput()
        self.__send(uploader.GET_SYNC +
                    uploader.EOC)
        self.__getSync()

    def __trySync(self):
        try:
            self.port.flush()
            if (self.__recv() != self.INSYNC):
                # print("unexpected 0x%x instead of INSYNC" % ord(c))
                return False
            c = self.__recv()
            if (c == self.BAD_SILICON_REV):
                raise NotImplementedError()
            if (c != self.OK):
                # print("unexpected 0x%x instead of OK" % ord(c))
                return False
            return True

        except NotImplementedError:
            raise RuntimeError("Programing not supported for this version of silicon!\n"
                               "See https://docs.px4.io/master/en/flight_controller/silicon_errata.html")
        except RuntimeError:
            # timeout, no response yet
            return False

    # attempt to determins if the device is CDCACM or A FTDI
    def __determineInterface(self):
        self.port.flushInput()
        # Set a baudrate that can not work on a real serial port
        # in that it is 233% off.
        try:
            self.port.baudrate = self.baudrate_bootloader * 2.33
        except NotImplementedError as e:
            # This error can occur because pySerial on Windows does not support odd baudrates
            print(str(e) + " -> could not check for FTDI device, assuming USB connection")
            return

        self.__send(uploader.GET_SYNC +
                    uploader.EOC)
        try:
            self.__getSync(False)
        except:
            # if it fails we are on a real serial port - only leave this enabled on Windows
            if sys.platform.startswith('win'):
                self.ackWindowedMode = True
        finally:
            self.port.baudrate = self.baudrate_bootloader

    # send the GET_DEVICE command and wait for an info parameter
    def __getInfo(self, param):
        self.__send(uploader.GET_DEVICE + param + uploader.EOC)
        value = self.__recv_int()
        self.__getSync()
        return value

    # send the GET_OTP command and wait for an info parameter
    def __getOTP(self, param):
        t = struct.pack("I", param)  # int param as 32bit ( 4 byte ) char array.
        self.__send(uploader.GET_OTP + t + uploader.EOC)
        value = self.__recv(4)
        self.__getSync()
        return value

    # send the GET_SN command and wait for an info parameter
    def __getSN(self, param):
        t = struct.pack("I", param)  # int param as 32bit ( 4 byte ) char array.
        self.__send(uploader.GET_SN + t + uploader.EOC)
        value = self.__recv(4)
        self.__getSync()
        return value

    # send the GET_CHIP command
    def __getCHIP(self):
        self.__send(uploader.GET_CHIP + uploader.EOC)
        value = self.__recv_int()
        self.__getSync()
        return value

    # send the GET_CHIP command
    def __getCHIPDes(self):
        self.__send(uploader.GET_CHIP_DES + uploader.EOC)
        length = self.__recv_int()
        value = self.__recv(length)
        self.__getSync()
        pieces = value.split(b",")
        return pieces

    def __drawProgressBar(self, label, progress, maxVal):
        if maxVal < progress:
            progress = maxVal

        percent = (float(progress) / float(maxVal)) * 100.0

        sys.stdout.write("\r%s: [%-20s] %.1f%%" % (label, '='*int(percent/5.0), percent))
        sys.stdout.flush()

    # send the CHIP_ERASE command and wait for the bootloader to become ready
    def __erase(self, label):
        print("Windowed mode: %s" % self.ackWindowedMode)
        print("\n", end='')
        self.__send(uploader.CHIP_ERASE +
                    uploader.EOC)

        # erase is very slow, give it 30s
        deadline = time.time() + 30.0
        while time.time() < deadline:

            usualEraseDuration = 15.0
            estimatedTimeRemaining = deadline-time.time()
            if estimatedTimeRemaining >= usualEraseDuration:
                self.__drawProgressBar(label, 30.0-estimatedTimeRemaining, usualEraseDuration)
            else:
                self.__drawProgressBar(label, 10.0, 10.0)
                sys.stdout.write(" (timeout: %d seconds) " % int(deadline-time.time()))
                sys.stdout.flush()

            if self.__trySync():
                self.__drawProgressBar(label, 10.0, 10.0)
                return

        raise RuntimeError("timed out waiting for erase")

    # send a PROG_MULTI command to write a collection of bytes
    def __program_multi(self, data, windowMode):

        length = len(data).to_bytes(1, byteorder='big')

        self.__send(uploader.PROG_MULTI)
        self.__send(length)
        self.__send(data)
        self.__send(uploader.EOC)
        if (not windowMode):
            self.__getSync(False)
        else:
            # The following is done to have minimum delay on the transmission
            # of the ne fw. The per block cost of __getSync was about 16 mS per.
            # Passively wait on Sync and Result using board rates and
            # N.B. attempts to activly wait on InWating still carried 8 mS of overhead
            self.__probe(False)
            self.__probe(True)
            time.sleep((ord(length) * self.chartime) + uploader.MAX_FLASH_PRGRAM_TIME)
            self.__probe(False)

    # verify multiple bytes in flash
    def __verify_multi(self, data):

        length = len(data).to_bytes(1, byteorder='big')

        self.__send(uploader.READ_MULTI)
        self.__send(length)
        self.__send(uploader.EOC)
        self.port.flush()
        programmed = self.__recv(len(data))
        if programmed != data:
            print("got    " + binascii.hexlify(programmed))
            print("expect " + binascii.hexlify(data))
            return False
        self.__getSync()
        return True

    # send the reboot command
    def __reboot(self):
        self.__send(uploader.REBOOT +
                    uploader.EOC)
        self.port.flush()

        # v3+ can report failure if the first word flash fails
        if self.bl_rev >= 3:
            self.__getSync()

    # split a sequence into a list of size-constrained pieces
    def __split_len(self, seq, length):
        return [seq[i:i+length] for i in range(0, len(seq), length)]

    # upload code
    def __program(self, label, fw):
        self.__probe(False)
        print("\n", end='')
        code = fw.image
        groups = self.__split_len(code, uploader.PROG_MULTI_MAX)
        # Give imedate feedback
        self.__drawProgressBar(label, 0, len(groups))
        uploadProgress = 0
        for bytes in groups:
            self.__program_multi(bytes, self.ackWindowedMode)
            # If in Window mode, extend the window size for the __ackSyncWindow
            if self.ackWindowedMode:
                self.window += self.window_per

            # Print upload progress (throttled, so it does not delay upload progress)
            uploadProgress += 1
            if uploadProgress % 256 == 0:
                self.__probe(True)
                self.__probe(False)
                self.__probe(True)
                self.__ackSyncWindow(self.window)
                self.__probe(False)
                self.window = 0
                self.__drawProgressBar(label, uploadProgress, len(groups))

        # Do any remaining fragment
        self.__ackSyncWindow(self.window)
        self.window = 0
        self.__drawProgressBar(label, 100, 100)

    # verify code
    def __verify_v2(self, label, fw):
        print("\n", end='')
        self.__send(uploader.CHIP_VERIFY +
                    uploader.EOC)
        self.__getSync()
        code = fw.image
        groups = self.__split_len(code, uploader.READ_MULTI_MAX)
        verifyProgress = 0
        for bytes in groups:
            verifyProgress += 1
            if verifyProgress % 256 == 0:
                self.__drawProgressBar(label, verifyProgress, len(groups))
            if (not self.__verify_multi(bytes)):
                raise RuntimeError("Verification failed")
        self.__drawProgressBar(label, 100, 100)

    def __verify_v3(self, label, fw):
        print("\n", end='')
        self.__drawProgressBar(label, 1, 100)
        expect_crc = fw.crc(self.fw_maxsize)
        self.__send(uploader.GET_CRC + uploader.EOC)
        time.sleep(0.5)
        report_crc = self.__recv_int()
        self.__getSync()
        if report_crc != expect_crc:
            print("Expected 0x%x" % expect_crc)
            print("Got      0x%x" % report_crc)
            raise RuntimeError("Program CRC failed")
        self.__drawProgressBar(label, 100, 100)

    def __set_boot_delay(self, boot_delay):
        self.__send(uploader.SET_BOOT_DELAY +
                    struct.pack("b", boot_delay) +
                    uploader.EOC)
        self.__getSync()

    # get basic data about the board
    def identify(self):
        self.__determineInterface()
        # make sure we are in sync before starting
        self.__sync()

        # get the bootloader protocol ID first
        self.bl_rev = self.__getInfo(uploader.INFO_BL_REV)
        if (self.bl_rev < uploader.BL_REV_MIN) or (self.bl_rev > uploader.BL_REV_MAX):
            print("Unsupported bootloader protocol %d" % uploader.INFO_BL_REV)
            raise RuntimeError("Bootloader protocol mismatch")

        self.board_type = self.__getInfo(uploader.INFO_BOARD_ID)
        self.board_rev = self.__getInfo(uploader.INFO_BOARD_REV)
        self.fw_maxsize = self.__getInfo(uploader.INFO_FLASH_SIZE)

    # upload the firmware
    def upload(self, fw, force=False, boot_delay=None):
        # Make sure we are doing the right thing
        start = time.time()
        if self.board_type != fw.property('board_id'):
            msg = "Firmware not suitable for this board (Firmware board_type=%u board_id=%u)" % (
                self.board_type, fw.property('board_id'))
            print("WARNING: %s" % msg)
            if force:
                print("FORCED WRITE, FLASHING ANYWAY!")
            else:
                raise FirmwareNotSuitableException(msg)

        # Prevent uploads where the image would overflow the flash
        if self.fw_maxsize < fw.property('image_size'):
            raise RuntimeError("Firmware image is too large for this board")

        # OTP added in v4:
        if self.bl_rev >= 4:
            for byte in range(0, 32*6, 4):
                x = self.__getOTP(byte)
                self.otp = self.otp + x
                # print(binascii.hexlify(x).decode('Latin-1') + ' ', end='')
            # see src/modules/systemlib/otp.h in px4 code:
            self.otp_id = self.otp[0:4]
            self.otp_idtype = self.otp[4:5]
            self.otp_vid = self.otp[8:4:-1]
            self.otp_pid = self.otp[12:8:-1]
            self.otp_coa = self.otp[32:160]
            # show user:
            try:
                print("sn: ", end='')
                for byte in range(0, 12, 4):
                    x = self.__getSN(byte)
                    x = x[::-1]  # reverse the bytes
                    self.sn = self.sn + x
                    print(binascii.hexlify(x).decode('Latin-1'), end='')  # show user
                print('')
                print("chip: %08x" % self.__getCHIP())

                otp_id = self.otp_id.decode('Latin-1')
                if ("PX4" in otp_id):
                    print("OTP id: " + otp_id)
                    print("OTP idtype: " + binascii.b2a_qp(self.otp_idtype).decode('Latin-1'))
                    print("OTP vid: " + binascii.hexlify(self.otp_vid).decode('Latin-1'))
                    print("OTP pid: " + binascii.hexlify(self.otp_pid).decode('Latin-1'))
                    print("OTP coa: " + binascii.b2a_base64(self.otp_coa).decode('Latin-1'))

            except Exception:
                # ignore bad character encodings
                pass

        # Silicon errata check was added in v5
        if (self.bl_rev >= 5):
            des = self.__getCHIPDes()
            if (len(des) == 2):
                print("family: %s" % des[0])
                print("revision: %s" % des[1])
                print("flash: %d bytes" % self.fw_maxsize)

                # Prevent uploads where the maximum image size of the board config is smaller than the flash
                # of the board. This is a hint the user chose the wrong config and will lack features
                # for this particular board.

                # This check should also check if the revision is an unaffected revision
                # and thus can support the full flash, see
                # https://github.com/PX4/Firmware/blob/master/src/drivers/boards/common/stm32/board_mcu_version.c#L125-L144

                if self.fw_maxsize > fw.property('image_maxsize') and not force:
                    raise RuntimeError("Board can accept larger flash images (%u bytes) than board config (%u bytes). Please use the correct board configuration to avoid lacking critical functionality."
                                       % (self.fw_maxsize, fw.property('image_maxsize')))
        else:
            # If we're still on bootloader v4 on a Pixhawk, we don't know if we
            # have the silicon errata and therefore need to flash px4_fmu-v2
            # with 1MB flash or if it supports px4_fmu-v3 with 2MB flash.
            if fw.property('board_id') == 9 \
                    and fw.property('image_size') > 1032192 \
                    and not force:
                raise RuntimeError("\nThe Board uses bootloader revision 4 and can therefore not determine\n"
                                   "if flashing more than 1 MB (px4_fmu-v3_default) is safe, chances are\n"
                                   "high that it is not safe! If unsure, use px4_fmu-v2_default.\n"
                                   "\n"
                                   "If you know you that the board does not have the silicon errata, use\n"
                                   "this script with --force, or update the bootloader. If you are invoking\n"
                                   "upload using make, you can use force-upload target to force the upload.\n")
        self.__erase("Erase  ")
        self.__program("Program", fw)

        if self.bl_rev == 2:
            self.__verify_v2("Verify ", fw)
        else:
            self.__verify_v3("Verify ", fw)

        if boot_delay is not None:
            self.__set_boot_delay(boot_delay)

        print("\nRebooting.", end='')
        self.__reboot()
        self.port.close()
        print(" Elapsed Time %3.3f\n" % (time.time() - start))

    def __next_baud_flightstack(self):
        if self.baudrate_flightstack_idx + 1 >= len(self.baudrate_flightstack):
            return False
        try:
            self.port.baudrate = self.baudrate_flightstack[self.baudrate_flightstack_idx + 1]
            self.baudrate_flightstack_idx = self.baudrate_flightstack_idx + 1
        except serial.SerialException:
            # Sometimes _configure_port fails
            time.sleep(0.04)

        return True

    def send_reboot(self):
        if (not self.__next_baud_flightstack()):
            return False

        print("Attempting reboot on %s with baudrate=%d..." % (self.port.port, self.port.baudrate), file=sys.stderr)
        if "ttyS" in self.port.port:
            print("If the board does not respond, check the connection to the Flight Controller")
        else:
            print("If the board does not respond, unplug and re-plug the USB connector.", file=sys.stderr)

        try:
            # try MAVLINK command first
            self.port.flush()
            self.__send(uploader.MAVLINK_REBOOT_ID1)
            self.__send(uploader.MAVLINK_REBOOT_ID0)
            # then try reboot via NSH
            self.__send(uploader.NSH_INIT)
            self.__send(uploader.NSH_REBOOT_BL)
            self.__send(uploader.NSH_INIT)
            self.__send(uploader.NSH_REBOOT)
            self.port.flush()
            self.port.baudrate = self.baudrate_bootloader
        except Exception:
            try:
                self.port.flush()
                self.port.baudrate = self.baudrate_bootloader
            except Exception:
                pass

        return True


def main():
    # Python2 is EOL
    if not runningPython3:
        raise RuntimeError("Python 2 is not supported. Please try again using Python 3.")

    # Parse commandline arguments
    parser = argparse.ArgumentParser(description="Firmware uploader for the PX autopilot system.")
    parser.add_argument('--port', action="store", required=True, help="Comma-separated list of serial port(s) to which the FMU may be attached")
    parser.add_argument('--baud-bootloader', action="store", type=int, default=115200, help="Baud rate of the serial port (default is 115200) when communicating with bootloader, only required for true serial ports.")
    parser.add_argument('--baud-flightstack', action="store", default="57600", help="Comma-separated list of baud rate of the serial port (default is 57600) when communicating with flight stack (Mavlink or NSH), only required for true serial ports.")
    parser.add_argument('--force', action='store_true', default=False, help='Override board type check, or silicon errata checks and continue loading')
    parser.add_argument('--boot-delay', type=int, default=None, help='minimum boot delay to store in flash')
    parser.add_argument('firmware', action="store", help="Firmware file to be uploaded")
    args = parser.parse_args()

    # warn people about ModemManager which interferes badly with Pixhawk
    if os.path.exists("/usr/sbin/ModemManager"):
        print("==========================================================================================================")
        print("WARNING: You should uninstall ModemManager as it conflicts with any non-modem serial device (like Pixhawk)")
        print("==========================================================================================================")

    # We need to check for pyserial because the import itself doesn't
    # seem to fail, at least not on macOS.
    pyserial_installed = False
    try:
        if serial.__version__:
            pyserial_installed = True
    except:
        pass

    try:
        if serial.VERSION:
            pyserial_installed = True
    except:
        pass

    if not pyserial_installed:
        print("Error: pyserial not installed!")
        print("")
        print("You may need to install it using:")
        print("    pip3 install --user pyserial")
        print("")
        sys.exit(1)

    # Load the firmware file
    fw = firmware(args.firmware)

    percent = fw.property('image_size') / fw.property('image_maxsize')
    binary_size = float(fw.property('image_size'))
    binary_max_size = float(fw.property('image_maxsize'))
    percent = (binary_size / binary_max_size) * 100

    print("Loaded firmware for board id: %s,%s size: %d bytes (%.2f%%), waiting for the bootloader..." % (fw.property('board_id'), fw.property('board_revision'), fw.property('image_size'), percent))
    print()

    # tell any GCS that might be connected to the autopilot to give up
    # control of the serial port

    # send to localhost and default GCS port
    ipaddr = '127.0.0.1'
    portnum = 14550

    # COMMAND_LONG in MAVLink 1
    heartbeatpacket = bytearray.fromhex('fe097001010000000100020c5103033c8a')
    commandpacket = bytearray.fromhex('fe210101014c00000000000000000000000000000000000000000000803f00000000f6000000008459')

    # initialize an UDP socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # send heartbeat to initialize connection and command to free the link
    s.sendto(heartbeatpacket, (ipaddr, portnum))
    s.sendto(commandpacket, (ipaddr, portnum))

    # close the socket
    s.close()

    # Spin waiting for a device to show up
    try:
        while True:
            portlist = []
            patterns = args.port.split(",")
            # on unix-like platforms use glob to support wildcard ports. This allows
            # the use of /dev/serial/by-id/usb-3D_Robotics on Linux, which prevents the upload from
            # causing modem hangups etc
            if "linux" in _platform or "darwin" in _platform or "cygwin" in _platform:
                import glob
                for pattern in patterns:
                    portlist += glob.glob(pattern)
            else:
                portlist = patterns

            baud_flightstack = [int(x) for x in args.baud_flightstack.split(',')]

            successful = False
            unsuitable_board = False
            for port in portlist:

                # print("Trying %s" % port)

                # create an uploader attached to the port
                try:
                    if "linux" in _platform:
                        # Linux, don't open Mac OS and Win ports
                        if "COM" not in port and "tty.usb" not in port:
                            up = uploader(port, args.baud_bootloader, baud_flightstack)
                    elif "darwin" in _platform:
                        # OS X, don't open Windows and Linux ports
                        if "COM" not in port and "ACM" not in port:
                            up = uploader(port, args.baud_bootloader, baud_flightstack)
                    elif "cygwin" in _platform:
                        # Cygwin, don't open native Windows COM and Linux ports
                        if "COM" not in port and "ACM" not in port:
                            up = uploader(port, args.baud_bootloader, baud_flightstack)
                    elif "win" in _platform:
                        # Windows, don't open POSIX ports
                        if "/" not in port:
                            up = uploader(port, args.baud_bootloader, baud_flightstack)
                except Exception:
                    # open failed, rate-limit our attempts
                    time.sleep(0.05)

                    # and loop to the next port
                    continue

                found_bootloader = False
                while True:
                    up.open()

                    # port is open, try talking to it
                    try:
                        # identify the bootloader
                        up.identify()
                        found_bootloader = True
                        print()
                        print("Found board id: %s,%s bootloader version: %s on %s" % (up.board_type, up.board_rev, up.bl_rev, port))
                        break

                    except Exception:

                        if not up.send_reboot():
                            break

                        # wait for the reboot, without we might run into Serial I/O Error 5
                        time.sleep(0.25)

                        # always close the port
                        up.close()

                        # wait for the close, without we might run into Serial I/O Error 6
                        time.sleep(0.3)

                if not found_bootloader:
                    # Go to the next port
                    continue

                try:
                    # ok, we have a bootloader, try flashing it
                    up.upload(fw, force=args.force, boot_delay=args.boot_delay)

                    # if we made this far without raising exceptions, the upload was successful
                    successful = True

                except RuntimeError as ex:
                    # print the error
                    print("\nERROR: %s" % ex.args)

                except FirmwareNotSuitableException:
                    unsuitable_board = True
                    up.close()
                    continue

                except IOError:
                    up.close()
                    continue

                finally:
                    # always close the port
                    up.close()

                # we could loop here if we wanted to wait for more boards...
                if successful:
                    sys.exit(0)
                else:
                    sys.exit(1)

            if unsuitable_board:
                # If we land here, we went through all ports, did not flash any
                # board and found at least one unsuitable board.
                # Exit with 2, so a caller can distinguish from other errors
                sys.exit(2)

            # Delay retries to < 20 Hz to prevent spin-lock from hogging the CPU
            time.sleep(0.05)

    # CTRL+C aborts the upload/spin-lock by interrupt mechanics
    except KeyboardInterrupt:
        print("\n Upload aborted by user.")
        sys.exit(0)


if __name__ == '__main__':
    main()

# vim: tabstop=4 expandtab shiftwidth=4 softtabstop=4
