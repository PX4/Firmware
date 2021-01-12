#! /usr/bin/env python3

import serial, time
import subprocess
from subprocess import call, Popen
from argparse import ArgumentParser
import re

def monitor_firmware_upload(port, baudrate):
    databits = serial.EIGHTBITS
    stopbits = serial.STOPBITS_ONE
    parity = serial.PARITY_NONE
    ser = serial.Serial(port, baudrate, databits, parity, stopbits, timeout=1)

    finished = 0

    timeout = 300  # 5 minutes
    timeout_start = time.time()
    timeout_newline = time.time()

    while finished == 0:
        serial_line = ser.readline().decode("ascii", errors='ignore')
        if (len(serial_line) > 0):
            print(serial_line.replace('\n', ''))

        if "NuttShell (NSH)" in serial_line:
            finished = 1
            break

        if time.time() - timeout_start > 10:
            if "nsh>" in serial_line:
                finished = 1
                break

        if time.time() > timeout_start + timeout:
            print("Error, timeout")
            finished = 1
            break

        # newline every 10 seconds if still running
        if time.time() - timeout_newline > 10:
            timeout_newline = time.time()
            ser.write('\n'.encode("ascii"))
            ser.flush()

    ser.close()

def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('--device', "-d", nargs='?', default=None, help='', required=True)
    parser.add_argument("--baudrate", "-b", dest="baudrate", type=int, help="Mavlink port baud rate (default=57600)", default=57600)
    args = parser.parse_args()

    monitor_firmware_upload(args.device, args.baudrate)

if __name__ == "__main__":
   main()
