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
from time import sleep, time

from cx10ds import CX10DS
from UavDetector import UavDetector
from UavController import UavController

from pyueye_camera import Camera
from pyueye_utils import FrameThread, FrameNoThread
from pyueye import ueye

from math import sqrt

import sys
from os import path, getenv
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python/")
from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage 


class Cx10dsUeye:
    def __init__(self, verbose=False):
        self.verbose = verbose
        self.step = 0.1 # period in seconds

        self.throttle = 128 # controls the throttle range 0-255
        self.rudder = 128 # controls the rudder range 48-208 ?? CHECK
        self.aileron = 128 #todo: range check
        self.elevator = 128 #todo: range check
        self.mode = 0 # 0 = idle, 1 = takeoff, 2 = land
        self.auto = False # manual control

        # Detector
        self._detector = UavDetector()

        # Controller
        self._ctrl = UavController(gui=True, detector=self._detector)

        # CX10 remote control
        self._cx10 = CX10DS()

        # Start IVY interface
        self._interface = IvyMessagesInterface("Cx10dsUeye")

        # Start camera
        self._thread = None
        try:
            self._cam = Camera()
            self._cam.init()
            #self._cam.set_colormode(ueye.IS_CM_BGR8_PACKED)
            self._cam.set_colormode(ueye.IS_CM_MONO8)
            #self._cam.set_aoi(0, 0, 1600, 1200)
            self._cam.set_aoi(0, 0, 1936, 1216)
            #self._cam.set_pixel_clock(30)
            self._cam.set_fps(10.)
            self._cam.set_exposure(1.)
            self._cam.alloc(6)
            self._cam.capture_video(wait=True)
            print("cam started")
        except Exception as e:
            print("cam failed to open, stopping", str(e))
            self.stop()
            sleep(1)
            exit(1)

        # a thread that waits for new images and call processing
        self._thread = FrameNoThread(self._cam, self._detector.process, show=True)
        self._thread.start()

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
            print("stop Ivy")
        if self._thread is not None and self._thread.running:
            self._thread.stop()
            self._thread.join()
            self._cam.stop_video()
            self._cam.exit()
            print("stop thread and cam")

    # main loop
    def run(self):
        #i = 0
        #size = 0.
        #nb = 100
        try:
            last_time = time()
            while True:
                valid = False
                self._thread.run_once()
                if self._detector.new_data:
                    x, y, a = self._detector.x, self._detector.y, self._detector.area
                    self._detector.new_data = False
                    valid = True
                    if a < 1.:
                        a = 1.
                    #dist = sqrt(1200/a)
                    dist = 400 / sqrt(a)
                    if self.verbose:
                        print('x: {:0.2f}, y: {:0.2f}, a: {:0.4f}, d: {:0.2f}'.format(x,y,a,dist))
                        #i += 1
                        #size += a
                        #if i == nb:
                        #    print("area {}".format(size/nb))
                        #    i = 0
                        #    size = 0.
                if self.auto:
                    if valid:
                        (r, p, y, t) = self._ctrl.run(x,y,dist)
                        self._cx10.set_cmd(r,p,y,t,0)
                        #self._cx10.set_cmd(r,self.elevator,y,t,0)
                        if self.verbose:
                            print("auto",r,p,y,t,a)
                else:
                    self._ctrl.reset()
                    self._cx10.set_cmd(self.aileron,self.elevator,self.rudder,self.throttle,self.mode)
                current_time = time()
                dt = current_time - last_time
                if dt >= self.step:
                    self._cx10.send()
                    last_time = current_time
                key = self._ctrl.refresh()
                if key == ord('c'):
                    self._detector.clearmask()
                elif key == ord('s'):
                    self._detector.setmask()
                elif key == ord('t'):
                    self._cx10.set_trim()
                elif key == ord('m'):
                    self._ctrl.start_stop_mission()
                elif key == ord('q'):
                    if self.verbose:
                        print("Exiting..")
                    self.stop()
                    break

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

    rmt = Cx10dsUeye(verbose=args.verbose)
    rmt.run()

