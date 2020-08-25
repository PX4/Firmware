#! /usr/bin/python

import serial, time
import subprocess
from subprocess import call, Popen
from argparse import ArgumentParser
import re
import unittest
import os

def do_test(port, baudrate, test_name):
    databits = serial.EIGHTBITS
    stopbits = serial.STOPBITS_ONE
    parity = serial.PARITY_NONE
    ser = serial.Serial(port, baudrate, databits, parity, stopbits, timeout=10)

    ser.write('\n\n')

    finished = 0
    success = False

    timeout = 10  # 10 seconds
    timeout_start = time.time()

    while finished == 0:
        serial_line = ser.readline()
        print(serial_line.replace('\n',''))

        if "nsh>" in serial_line:
            finished = 1

        if time.time() > timeout_start + timeout:
            print("Error, timeout")
            finished = 1
            break


    # run test
    ser.write('tests ' + test_name + '\n')
    time.sleep(0.05)

    finished = 0
    timeout = 300  # 5 minutes
    timeout_start = time.time()
    timeout_newline = time.time()

    while finished == 0:
        serial_line = ser.readline()
        print(serial_line.replace('\n',''))

        if test_name + " PASSED" in serial_line:
            finished = 1
            success = True
        elif test_name + " FAILED" in serial_line:
            finished = 1
            success = False

        if time.time() > timeout_start + timeout:
            print("Error, timeout")
            print(test_name + " FAILED")
            finished = 1
            success = False
            break

        # newline every 30 seconds if still running
        if time.time() - timeout_newline > 30:
            ser.write('\n')
            timeout_newline = time.time()

    ser.close()

    return success

class TestHadrwareMethods(unittest.TestCase):
    TEST_DEVICE = 0
    TEST_BAUDRATE = 0

    def test_autodeclination(self):
        print("runnig test device " + self.TEST_DEVICE)
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "autodeclination"))

    def test_bezier(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "bezier"))

    def test_bson(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "bson"))

    def test_commander(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "commander"))

    def test_controllib(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "controllib"))

#    def test_dataman(self):
#        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "dataman"))

    def floattest_float(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "float"))

    def hrttest_hrt(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "hrt"))

    def test_hrt(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "hrt"))

    def test_IntrusiveQueue(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "IntrusiveQueue"))

    def test_List(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "List"))

    def test_mathlib(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "mathlib"))

    def test_matrix(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "matrix"))

    def test_microbench_hrt(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "microbench_hrt"))

    def test_microbench_math(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "microbench_math"))

    def test_microbench_matrix(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "microbench_matrix"))

    def test_microbench_uorb(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "microbench_uorb"))

#    def test_mixer(self):
#        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "mixer"))

    def test_param(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "param"))

    def test_parameters(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "parameters"))

    def test_perf(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "perf"))

    def search_mintest_xxx(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "search_min"))

    def test_sleep(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "sleep"))

    def test_smoothz(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "smoothz"))

    def test_time(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "time"))

    #def test_uorb(self):
    #    self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "uorb"))

    def test_versioning(self):
        self.assertTrue(do_test(self.TEST_DEVICE, self.TEST_BAUDRATE, "versioning"))

def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('--device', "-d", nargs='?', default = None, help='')
    parser.add_argument("--baudrate", "-b", dest="baudrate", type=int, help="Mavlink port baud rate (default=57600)", default=57600)
    args = parser.parse_args()

    TestHadrwareMethods.TEST_DEVICE = args.device
    TestHadrwareMethods.TEST_BAUDRATE =  args.baudrate

    unittest.main(__name__, argv=['main'])

if __name__ == "__main__":
   main()
