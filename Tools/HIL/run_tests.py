#! /usr/bin/env python3

import serial, time
import subprocess
from subprocess import call, Popen
from argparse import ArgumentParser
import re
import unittest
import os
import sys

def do_test(port, baudrate, test_name):
    databits = serial.EIGHTBITS
    stopbits = serial.STOPBITS_ONE
    parity = serial.PARITY_NONE
    ser = serial.Serial(port, baudrate, databits, parity, stopbits, timeout=1)

    success = False

    # run test cmd
    print('\n|======================================================================')
    cmd = 'tests ' + test_name
    print("| Running:", cmd)
    print('|======================================================================')
    timeout_start = time.time()
    timeout = 10  # 10 seconds

    # clear
    ser.write("\n".encode("ascii"))
    ser.flush()
    ser.readline()

    serial_cmd = '{0}\n'.format(cmd)
    ser.write(serial_cmd.encode("ascii"))
    ser.flush()
    ser.readline()

    # TODO: retry command
    # while True:
    #     serial_cmd = '{0}\n'.format(cmd)
    #     ser.write(serial_cmd.encode("ascii"))
    #     ser.flush()

    #     serial_line = ser.readline().decode("ascii", errors='ignore')

    #     if cmd in serial_line:
    #         break
    #     else:
    #         print(serial_line.replace('\n', ''))

    #     if time.time() > timeout_start + timeout:
    #         print("Error, unable to write cmd")
    #         return False

    #     time.sleep(1)


    # print results, wait for final result (PASSED or FAILED)
    timeout = 180  # 3 minutes
    timeout_start = time.time()
    timeout_newline = timeout_start

    while True:
        serial_line = ser.readline().decode("ascii", errors='ignore')
        if (len(serial_line) > 0):
            print(serial_line, end='')

        if test_name + " PASSED" in serial_line:
            success = True
            break
        elif test_name + " FAILED" in serial_line:
            success = False
            break

        if time.time() > timeout_start + timeout:
            print("Error, timeout")
            print(test_name + " FAILED")
            success = False
            break

        # newline every 10 seconds if still running
        if time.time() - timeout_newline > 10:
            ser.write("\n".encode("ascii"))
            timeout_newline = time.time()

    ser.close()

    return success

class TestHardwareMethods(unittest.TestCase):
    TEST_DEVICE = 0
    TEST_BAUDRATE = 0

    def test_atomic_bitset(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "atomic_bitset"))

    def test_bezier(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "bezier"))

    def test_bitset(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "bitset"))

    def test_bson(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "bson"))

    # def test_dataman(self):
    #     self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "dataman"))

    def floattest_float(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "float"))

    def test_hrt(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "hrt"))

    def test_IntrusiveQueue(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "IntrusiveQueue"))

    def test_IntrusiveSortedList(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "IntrusiveSortedList"))

    def test_List(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "List"))

    def test_mathlib(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "mathlib"))

    def test_matrix(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "matrix"))

    def test_microbench_atomic(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "microbench_atomic"))

    def test_microbench_hrt(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "microbench_hrt"))

    def test_microbench_math(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "microbench_math"))

    def test_microbench_matrix(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "microbench_matrix"))

    def test_microbench_uorb(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "microbench_uorb"))

    # def test_mixer(self):
    #     self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "mixer"))

    def test_param(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "param"))

    def test_parameters(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "parameters"))

    def test_perf(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "perf"))

    # def test_rc(self):
    #     self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "rc"))

    def test_search_min(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "search_min"))

    def test_sleep(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "sleep"))

    def test_time(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "time"))

    def test_versioning(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "versioning"))

def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('--device', "-d", nargs='?', default=None, help='', required=True)
    parser.add_argument("--baudrate", "-b", dest="baudrate", type=int, help="Mavlink port baud rate (default=57600)", default=57600)
    args = parser.parse_args()

    TestHardwareMethods.TEST_DEVICE = args.device
    TestHardwareMethods.TEST_BAUDRATE = args.baudrate

    unittest.main(__name__, failfast=True, verbosity=0, argv=['main'])

if __name__ == "__main__":
   main()
