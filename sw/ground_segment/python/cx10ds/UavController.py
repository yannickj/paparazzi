#!/usr/bin/python
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

# Control the position of a UAV

import sys
sys.path.insert(0, '/home/gautier/usr/lib/python2.7/dist-packages')
import cv2
import numpy as np
from PID import PID

class UavController:
    def __init__(self, gui=True, detector=None):
        self.use_gui = gui
        self.pid_lat = PID(P=0.2,D=0.2,I=0.01)
        self.pid_vert = PID(P=0.5,D=0.5,I=0.)
        self.pid_dist = PID(P=0.5,D=0.4,I=0.05)
        self.speed = 128 ## openloop forward control FIXME use size?

        if self.use_gui:
            print("start control GUI")
            cv2.namedWindow('ctrl')
            # create trackbars for color change
            cv2.createTrackbar('P lat','ctrl',int(self.pid_lat.Kp*1000),1000,lambda x: self.pid_lat.setKp(x/1000.))
            cv2.createTrackbar('I lat','ctrl',int(self.pid_lat.Ki*1000),1000,lambda x: self.pid_lat.setKi(x/1000.))
            cv2.createTrackbar('D lat','ctrl',int(self.pid_lat.Kd*1000),1000,lambda x: self.pid_lat.setKd(x/1000.))
            cv2.createTrackbar('P vert','ctrl',int(self.pid_vert.Kp*1000),1000,lambda x: self.pid_vert.setKp(x/1000.))
            cv2.createTrackbar('I vert','ctrl',int(self.pid_vert.Ki*1000),1000,lambda x:  self.pid_vert.setKi(x/1000.))
            cv2.createTrackbar('D vert','ctrl',int(self.pid_vert.Kd*1000),1000,lambda x:  self.pid_vert.setKd(x/1000.))
            cv2.createTrackbar('Speed','ctrl',self.speed,255,self.set_speed)
            cv2.createTrackbar('P dist','ctrl',int(self.pid_dist.Kp*1000),1000,lambda x:  self.pid_dist.setKp(x/1000.))
            cv2.createTrackbar('I dist','ctrl',int(self.pid_dist.Ki*1000),1000,lambda x:  self.pid_dist.setKi(x/1000.))
            cv2.createTrackbar('D dist','ctrl',int(self.pid_dist.Kd*1000),1000,lambda x:  self.pid_dist.setKd(x/1000.))
            if detector is not None:
                cv2.createTrackbar('Thres', 'ctrl',detector.threshold,255,detector.set_thres)
            im = cv2.imread('cx10ds.jpg',cv2.IMREAD_COLOR)
            cv2.imshow('ctrl',im)
            cv2.waitKey(1000)


    def stop(self):
        if self.use_gui:
            cv2.destroyAllWindows()

    def set_speed(self, speed):
        self.speed = speed

    def run(self, lat, vert, dist):
        self.pid_lat.update(lat)
        self.pid_vert.update(vert)
        self.pid_dist.update(dist)
        # return (roll, pitch, yaw, thrust)
        return (127+int(self.pid_lat.output), self.speed, 127, 127+int(self.pid_vert.output))

    def reset(self):
        self.pid_lat.clear_PID()
        self.pid_vert.clear_PID()
        self.pid_dist.clear_PID()

    def refresh(self):
        cv2.waitKey(1)

