#!/usr/bin/env python
#
# MIT License
#
# Copyright (c) 2018 Nagy Arpad Peter
# Copyright (c) 2018 Gautier Hattenberger (for Paparazzi UAV)
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#

import socket
from time import sleep

import sys
from os import path, getenv
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python/")
from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage 

step = 5

class RemoteCX10DS:
    def __init__(self, verbose=False):
        self.verbose = verbose
        self.step = 0.1 # period in seconds
        self.button_land = False
        self.button_takeoff = False
        self.button_idle = False

        self.IP = "192.168.4.1"
        self.PORT = 8033
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.header = b'\xcc' #first byte of every payload
        self.footer = b'\x33' # last byte of every payload
        self.message = ''

        self.throttle = 128 # controls the throttle range 0-255
        self.rudder = 128 # controls the rudder range 48-208
        self.aileron = 128 #todo: range check
        self.elevator = 128 #todo: range check
        self.crc = 0 #crc calculated from thr, rdr, ail, elev, mode xor product
        self.mode = 0 # 0 = idle, 1 = takeoff, 2 = land

        # Start IVY interface
        self._interface = IvyMessagesInterface("CX10DS remote")

        # bind to JOYSTICK message
        def joystick_cb(ac_id, msg):
            self.aileron = self.valid_range(int(msg['axis1']))
            self.elevator = self.valid_range(int(msg['axis2']))
            self.rudder = self.valid_range(int(msg['axis3']))
            self.throttle = 128
            self.dir = int(msg['button1'])-1
            self.throttle_incr = self.valid_range(int(msg['axis4']), 0, 127)
            self.throttle = self.valid_range(128 + self.dir * self.throttle_incr) # up
            if msg['button2'] == '1' and not self.button_takeoff:
                self.mode = 1 # takeoff
            if msg['button3'] == '1' and not self.button_land:
                self.mode = 2 # land
            if msg['button4'] == '1' and not self.button_idle:
                self.mode = 0 # idle
            self.button_land = (msg['button2'] == '1')
            self.button_takeoff = (msg['button3'] == '1')
            self.button_idle = (msg['button4'] == '1')
            #if msg['button1'] == '1' and not self.button_land:
            #    self.mode = 2 # land
            #if msg['button2'] == '1' and not self.button_takeoff:
            #    self.mode = 1 # takeoff
            #if msg['button3'] == '1':
            #    self.throttle = self.valid_range(128 + self.throttle_incr) # up
            #if msg['button4'] == '1':
            #    self.throttle = self.valid_range(128 - self.throttle_incr) # down
            #self.button_land = (msg['button1'] == '1')
            #self.button_takeoff = (msg['button2'] == '1')
            if self.verbose:
                print("Throttle {0}".format(self.throttle))
                print("Rudder {0}".format(self.rudder))
                print("elevator {0}".format(self.elevator))
                print("aileron {0}".format(self.aileron))
                print("Mode {0}".format(self.mode))

        self._interface.subscribe(joystick_cb, PprzMessage("ground", "JOYSTICK"))

    def __del__(self):
        self.stop()

    def stop(self):
        # Stop IVY interface
        if self._interface is not None:
            self._interface.shutdown()

    #sends out a message to the given IP/PORT every 0.5 seconds
    def send(self):
        self.createMSG()
        self.sock.sendto(self.message, (self.IP, self.PORT))
        if self.verbose:
            print(":".join(x.encode('hex') for x in self.message))

    # crc calculation
    def crc_calculate(self):
        self.crc = self.throttle ^ self.rudder ^ self.aileron ^ self.elevator ^ self.mode

    # create the message from the actual values of the virtual remote, must be called before each send
    def createMSG(self):
        self.crc_calculate()
        data = chr(self.aileron) + chr(self.elevator) + chr(self.throttle) + chr(self.rudder) + chr(self.mode) + chr(self.crc)

        self.message = self.header + data + self.footer

    # validate range with constraints
    def valid_range(self, value, MIN = 0, MAX = 255):
        return max(min(MAX, value), MIN)

    # main loop
    def run(self):
        try:
            while True:
                # TODO: make better frequency managing
                self.send()
                sleep(self.step)

        except KeyboardInterrupt:
            if self.verbose:
                print("Exiting..")
            self.stop()


# example for using the class
if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="CX10DS Remote Control")
    parser.add_argument('-v', '--verbose', dest='verbose', default=False, action='store_true', help="display debug messages")
    args = parser.parse_args()

    rmt = RemoteCX10DS(verbose=args.verbose)
    rmt.run()

