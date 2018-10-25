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
        self.pid_lat = PID(P=0.44,D=0.23,I=0.3)
        self.pid_vert = PID(P=1.3,D=0.5,I=0.35)
        self.pid_dist = PID(P=10.,D=4.,I=2.)
        self.mission = False
        self.autoland = False
        self.ff_speed_gain = 30.
        self.speed = 1. # in meters/s
        self.set_dist(4.) # in meters
        self.limit1 = 17. # in meters, end mission 1 limit
        self.limit2 = 10. # in meters, end mission 2 limit
        self.limit = self.limit1 # current limit

        if self.use_gui:
            print("start control GUI")
            cv2.namedWindow('ctrl')
            # create trackbars for color change
            cv2.createTrackbar('P lat','ctrl',int(self.pid_lat.Kp*1000),2000,lambda x: self.pid_lat.setKp(x/1000.))
            cv2.createTrackbar('I lat','ctrl',int(self.pid_lat.Ki*1000),2000,lambda x: self.pid_lat.setKi(x/1000.))
            cv2.createTrackbar('D lat','ctrl',int(self.pid_lat.Kd*1000),2000,lambda x: self.pid_lat.setKd(x/1000.))
            cv2.createTrackbar('P vert','ctrl',int(self.pid_vert.Kp*1000),2000,lambda x: self.pid_vert.setKp(x/1000.))
            cv2.createTrackbar('I vert','ctrl',int(self.pid_vert.Ki*1000),2000,lambda x:  self.pid_vert.setKi(x/1000.))
            cv2.createTrackbar('D vert','ctrl',int(self.pid_vert.Kd*1000),2000,lambda x:  self.pid_vert.setKd(x/1000.))
            cv2.createTrackbar('P dist','ctrl',int(self.pid_dist.Kp*1000),20000,lambda x:  self.pid_dist.setKp(x/1000.))
            cv2.createTrackbar('I dist','ctrl',int(self.pid_dist.Ki*1000),20000,lambda x:  self.pid_dist.setKi(x/1000.))
            cv2.createTrackbar('D dist','ctrl',int(self.pid_dist.Kd*1000),20000,lambda x:  self.pid_dist.setKd(x/1000.))
            cv2.createTrackbar('FF dist','ctrl',int(self.ff_speed_gain),100,lambda x:  self.set_ff_speed_gain(x))
            cv2.createTrackbar('Dist','ctrl',int(self.dist*10),300,lambda x: self.set_dist(x/10.))
            cv2.createTrackbar('Speed','ctrl',int(self.speed*10),30,lambda x: self.set_speed(x/10.))
            cv2.createTrackbar('Limit','ctrl',int(self.limit*10),300,lambda x: self.set_limit(x/10.))
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

    def set_ff_speed_gain(self, gain):
        self.ff_speed_gain = gain

    def set_dist(self, dist):
        self.dist = max(1.0, dist)
        if not self.mission:
            self.pid_dist.SetPoint = self.dist

    def set_limit(self, dist):
        self.limit1 = max(1.0, dist)
        print("set mission limit: {}".format(self.limit1))

    def set_limit2(self, dist):
        self.limit2 = max(1.0, dist)
        print("set mission 2 limit: {}".format(self.limit2))

    def run(self, lat, vert, dist, mode, in_flight=True):
        ff_cmd = 0
        new_mode = mode
        if new_mode == 2:
            self.land_mission()
        if self.mission:
            if self.pid_dist.SetPoint > self.limit:
                if self.autoland:
                    print("End mission and land")
                    new_mode = 2
                    self.land_mission()
                else:
                    print("Stop mission")
                    self.stop_mission()
            else:
                self.pid_dist.SetPoint += self.speed * 0.1 # FIXME should not be fixed dt
                ff_cmd = int(self.speed * self.ff_speed_gain)

        dist_coef = min(30.,max(1., dist)) # bound distance
        dist_coef = dist_coef / 5. # tune gains at 5 meters
        #print(dist, dist_coef)
        #print("lat")
        self.pid_lat.update(lat, in_flight, coef=min(1.2,dist_coef)) # coef = dist_coef ?
        #print("vert")
        self.pid_vert.update(vert, in_flight, coef=min(1.2,dist_coef))
        #print("dist")
        self.pid_dist.update(dist, in_flight, max_error=2.)#, 1./dist_coef) # coef = 1/dist_coef ?
        d_cmd = int(self.pid_dist.output)
        if dist > 15. and self.mission:
            d_cmd = 0
        # return (roll, pitch, yaw, thrust)
        print("mode",mode,new_mode)
        return (128+int(self.pid_lat.output), 128+ff_cmd+d_cmd, 128, 128+int(self.pid_vert.output),new_mode)

    def reset(self):
        self.pid_lat.clear_PID()
        self.pid_vert.clear_PID()
        self.pid_dist.clear_PID()

    def refresh(self):
        return cv2.waitKey(1) & 0xFF

    def start_stop_mission(self):
        if self.mission:
            # if running, stop and go back to start
            self.mission = False
            self.autoland = False
        else:
            # start mission
            self.limit = self.limit1
            self.mission = True
            self.autoland = False
        self.pid_dist.SetPoint = self.dist

    def start_stop_mission2(self):
        if self.mission:
            # if running, stop and go back to start
            self.mission = False
            self.autoland = False
        else:
            # start mission 2
            self.limit = self.limit2
            self.mission = True
            self.autoland = False
        self.pid_dist.SetPoint = self.dist

    def start_land_mission(self):
        self.start_stop_mission()
        self.autoland = True

    def start_land_mission2(self):
        self.start_stop_mission2()
        self.autoland = True

    def stop_mission(self):
        self.mission = False
        self.autoland = False
        self.pid_dist.SetPoint = self.dist # reset dist setpoint

    def land_mission(self):
        self.mission = False
        self.autoland = True

