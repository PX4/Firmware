#!/usr/bin/env python3

"""
Open a shell over MAVLink.

@author: Beat Kueng (beat-kueng@gmx.net)
"""


from __future__ import print_function
import sys, select
import termios
from timeit import default_timer as timer
from argparse import ArgumentParser

try:
    from pymavlink import mavutil
except ImportError as e:
    print("Failed to import pymavlink: " + str(e))
    print("")
    print("You may need to install it with:")
    print("    pip3 install --user pymavlink")
    print("")
    sys.exit(1)

try:
    import serial
except ImportError as e:
    print("Failed to import pyserial: " + str(e))
    print("")
    print("You may need to install it with:")
    print("    pip3 install --user pyserial")
    print("")
    sys.exit(1)


class MavlinkSerialPort():
    '''an object that looks like a serial port, but
    transmits using mavlink SERIAL_CONTROL packets'''
    def __init__(self, portname, baudrate, devnum=0, debug=0):
        self.baudrate = 0
        self._debug = debug
        self.buf = ''
        self.port = devnum
        self.debug("Connecting with MAVLink to %s ..." % portname)
        self.mav = mavutil.mavlink_connection(portname, autoreconnect=True, baud=baudrate)
        self.mav.wait_heartbeat()
        self.debug("HEARTBEAT OK\n")
        self.debug("Locked serial device\n")

    def debug(self, s, level=1):
        '''write some debug text'''
        if self._debug >= level:
            print(s)

    def write(self, b):
        '''write some bytes'''
        self.debug("sending '%s' (0x%02x) of len %u\n" % (b, ord(b[0]), len(b)), 2)
        while len(b) > 0:
            n = len(b)
            if n > 70:
                n = 70
            buf = [ord(x) for x in b[:n]]
            buf.extend([0]*(70-len(buf)))
            self.mav.mav.serial_control_send(self.port,
                                             mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                                             mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND,
                                             0,
                                             0,
                                             n,
                                             buf)
            b = b[n:]

    def close(self):
        self.mav.mav.serial_control_send(self.port, 0, 0, 0, 0, [0]*70)

    def _recv(self):
        '''read some bytes into self.buf'''
        m = self.mav.recv_match(condition='SERIAL_CONTROL.count!=0',
                                type='SERIAL_CONTROL', blocking=True,
                                timeout=0.03)
        if m is not None:
            if self._debug > 2:
                print(m)
            data = m.data[:m.count]
            self.buf += ''.join(str(chr(x)) for x in data)

    def read(self, n):
        '''read some bytes'''
        if len(self.buf) == 0:
            self._recv()
        if len(self.buf) > 0:
            if n > len(self.buf):
                n = len(self.buf)
            ret = self.buf[:n]
            self.buf = self.buf[n:]
            if self._debug >= 2:
                for b in ret:
                    self.debug("read 0x%x" % ord(b), 2)
            return ret
        return ''


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('port', metavar='PORT', nargs='?', default = None,
            help='Mavlink port name: serial: DEVICE[,BAUD], udp: IP:PORT, tcp: tcp:IP:PORT. Eg: \
/dev/ttyUSB0 or 0.0.0.0:14550. Auto-detect serial if not given.')
    parser.add_argument("--baudrate", "-b", dest="baudrate", type=int,
                      help="Mavlink port baud rate (default=57600)", default=57600)
    args = parser.parse_args()


    if args.port == None:
        if sys.platform == "darwin":
            args.port = "/dev/tty.usbmodem01"
        else:
            serial_list = mavutil.auto_detect_serial(preferred_list=['*FTDI*',
                "*Arduino_Mega_2560*", "*3D_Robotics*", "*USB_to_UART*", '*PX4*', '*FMU*', "*Gumstix*"])

            if len(serial_list) == 0:
                print("Error: no serial connection found")
                return

            if len(serial_list) > 1:
                print('Auto-detected serial ports are:')
                for port in serial_list:
                    print(" {:}".format(port))
            print('Using port {:}'.format(serial_list[0]))
            args.port = serial_list[0].device


    print("Connecting to MAVLINK...")
    mav_serialport = MavlinkSerialPort(args.port, args.baudrate, devnum=10)

    mav_serialport.write('\n') # make sure the shell is started

    # setup the console, so we can read one char at a time
    fd_in = sys.stdin.fileno()
    old_attr = termios.tcgetattr(fd_in)
    new_attr = termios.tcgetattr(fd_in)
    new_attr[3] = new_attr[3] & ~termios.ECHO # lflags
    new_attr[3] = new_attr[3] & ~termios.ICANON

    try:
        termios.tcsetattr(fd_in, termios.TCSANOW, new_attr)
        cur_line = ''
        command_history = []
        cur_history_index = 0

        def erase_last_n_chars(N):
            if N == 0: return
            CURSOR_BACK_N = '\x1b['+str(N)+'D'
            ERASE_END_LINE = '\x1b[K'
            sys.stdout.write(CURSOR_BACK_N + ERASE_END_LINE)

        next_heartbeat_time = timer()

        while True:
            while True:
                i, o, e = select.select([sys.stdin], [], [], 0)
                if not i: break
                ch = sys.stdin.read(1)

                # provide a simple shell with command history
                if ch == '\n':
                    if len(cur_line) > 0:
                        # erase current text (mavlink shell will echo it as well)
                        erase_last_n_chars(len(cur_line))

                        # add to history
                        if len(command_history) == 0 or command_history[-1] != cur_line:
                            command_history.append(cur_line)
                            if len(command_history) > 50:
                                del command_history[0]
                        cur_history_index = len(command_history)
                    mav_serialport.write(cur_line+'\n')
                    cur_line = ''
                elif ord(ch) == 127: # backslash
                    if len(cur_line) > 0:
                        erase_last_n_chars(1)
                        cur_line = cur_line[:-1]
                        sys.stdout.write(ch)
                elif ord(ch) == 27:
                    ch = sys.stdin.read(1) # skip one
                    ch = sys.stdin.read(1)
                    if ch == 'A': # arrow up
                        if cur_history_index > 0:
                            cur_history_index -= 1
                    elif ch == 'B': # arrow down
                        if cur_history_index < len(command_history):
                            cur_history_index += 1
                    # TODO: else: support line editing

                    erase_last_n_chars(len(cur_line))
                    if cur_history_index == len(command_history):
                        cur_line = ''
                    else:
                        cur_line = command_history[cur_history_index]
                    sys.stdout.write(cur_line)

                elif ord(ch) > 3:
                    cur_line += ch
                    sys.stdout.write(ch)
                sys.stdout.flush()

            data = mav_serialport.read(4096)
            if data and len(data) > 0:
                sys.stdout.write(data)
                sys.stdout.flush()

            # handle heartbeat sending
            heartbeat_time = timer()
            if heartbeat_time > next_heartbeat_time:
                mav_serialport.mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_GENERIC, 0, 0, 0)
                next_heartbeat_time = heartbeat_time + 1

    except serial.serialutil.SerialException as e:
        print(e)

    except KeyboardInterrupt:
        mav_serialport.close()

    finally:
        termios.tcsetattr(fd_in, termios.TCSADRAIN, old_attr)


if __name__ == '__main__':
    main()

