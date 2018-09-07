#!/usr/bin/env python
#
# MIT License
#
# Copyright (c) 2018 Gautier Hattenberger
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

from cx10ds import CX10DS
from UavController import UavController
import serial

import sys
from os import path, getenv
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python/")
from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage 


class Cx10dsJevois:
    def __init__(self, dev='/dev/ttyACM0', baud=115200, verbose=False):
        self.verbose = verbose
        self.step = 0.1 # period in seconds

        self.throttle = 128 # controls the throttle range 0-255
        self.rudder = 128 # controls the rudder range 48-208 ?? CHECK
        self.aileron = 128 #todo: range check
        self.elevator = 128 #todo: range check
        self.mode = 0 # 0 = idle, 1 = takeoff, 2 = land
        self.auto = False # manual control

        # controller
        self._ctrl = UavController()

        # CX10 remote control
        self._cx10 = CX10DS()

        # Start IVY interface
        self._interface = IvyMessagesInterface("Cx10dsJevois")

        # Open serial
        try:
            self._ser = serial.Serial(dev, baud, timeout=0.05)
            sleep(0.1)
            self._ser.reset_input_buffer()
        except:
            print("jevois serial not found")
            self.stop()
            exit(0)

        # bind to JOYSTICK message
        def joystick_cb(ac_id, msg):
            self.aileron = int(msg['axis1'])
            self.elevator = int(msg['axis2'])
            self.rudder = int(msg['axis3'])
            self.throttle = 128
            direction = int(msg['button1'])-1
            throttle_incr = self._cx10.valid_range(int(msg['axis4']), 0, 127)
            self.throttle = 128 + direction * throttle_incr # up
            self.mode = int(msg['button2'])
            self.auto = int(msg['button3']) == 1

        self._interface.subscribe(joystick_cb, PprzMessage("ground", "JOYSTICK"))

    def __del__(self):
        self.stop()

    def stop(self):
        # Stop IVY interface
        if self._interface is not None:
            self._interface.shutdown()

    # main loop
    def run(self):
        try:
            while True:
                try:
                    el = self._ser.readline().split(' ')
                except serial.SerialException, serial.SerialTimeoutException:
                    if self.verbose:
                        print("jevois readline error")
                valid = False
                if el[0] == "POS":
                    x, y, a = float(el[1]), float(el[2]), float(el[3])
                    valid = True
                    #dist = 8.152 * float(a)**(-0.2121) - 1.086
                    if self.verbose:
                        print('x: {}, y: {}, a: {}'.format(x,y,a))
                if self.auto:
                    if valid:
                        (r, p, y, t) = self._ctrl.run(x,y,a)
                        self._cx10.set_cmd(r,self.elevator,y,t,0)
                        if self.verbose:
                            print("auto",r,p,y,t,a)
                else:
                    self._ctrl.reset()
                    self._cx10.set_cmd(self.aileron,self.elevator,self.rudder,self.throttle,self.mode)
                self._cx10.send()
                self._ctrl.refresh()
                #sleep(self.step) # TODO better timing

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

    rmt = Cx10dsJevois(verbose=args.verbose)
    rmt.run()

