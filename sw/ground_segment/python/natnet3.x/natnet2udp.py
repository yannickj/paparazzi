#!/usr/bin/env python3
#
# Copyright (C) 2017 
#
# This file is part of paparazzi.
#
# paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi; see the file COPYING.  If not, see
# <http://www.gnu.org/licenses/>.
#

# python3 ./natnet2udo.py
if __debug__:
    from NatNetClient import NatNetClient
# python3 -O ./natnet2udp.py
else:
    import random
    import threading


import tkinter as tk
import socket
import time


NbParam = 6
RangeParam = [[-5.0, +5.0], [-5.0, +5.0], [0.0, +10.0], [-180.0, +180.0], [-180.0, +180.0], [-180.0, +180.0]]
FormatParam = ["{:+.3f}","{:+.3f}","{:+.3f}","{:+.1f}","{:+.1f}","{:+.1f}"]


class Mainwindow(tk.Frame):
    def __init__(self, parent):
        tk.Frame.__init__(self, parent)
        frameheight = (1080-70)/NbParam
        frames = [tk.Frame(parent) for j in range(NbParam)]
        self.linedata = [Linedata(frames[j],RangeParam[j],frameheight) for j in range(NbParam)]
        [frames[j].pack(fill="x") for j in range(NbParam)]

    def update(self,data_arr):
        [self.linedata[j].update_data(data_arr[j],FormatParam[j]) for j in range(NbParam)]


class Linedata():
    def __init__(self, parent, rangeparam, frameheight):
        self.resolution = frameheight / (rangeparam[1] - rangeparam[0])
        self.offset = ((rangeparam[1] + rangeparam[0]) / (rangeparam[1] - rangeparam[0]))*frameheight/2
        self.labeldata = tk.Label(parent, font=("Helvetica", 32), width=6)
        self.lineframe = Lineframe(parent,frameheight)
        self.lineframe.canvas.pack(side="left",fill="both",expand=True)
        self.labeldata.pack(side="left",fill="x")

    def update_data(self,data,formatparam):
        self.lineframe.update_plot(data * self.resolution - self.offset)
        self.labeldata.configure(text=formatparam.format(data))
        self.labeldata.update()


class Lineframe():
    def __init__(self, parent, frameheight):
        self.canvas = tk.Canvas(parent, background="black", height=frameheight)
        self.data_line = self.canvas.create_line(0,0,0,0, fill="red")

    def update_plot(self,value):
        self.add_point(self.data_line, value)
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))
        self.canvas.xview_moveto(1.0)

    def add_point(self, line, y):
        coords = self.canvas.coords(line)  # get line coordinates
        x = coords[-2] + 1                 # get x position starting from end
        coords.append(x)
        coords.append(y)
        coords = coords[-2000:]            # keep historic of points
        self.canvas.coords(line, *coords)  # update line coordinates


import math 
def quaternion_to_euler_angle(w, x, y, z):
       
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    X = math.degrees(math.atan2(t0, t1))
                    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.degrees(math.asin(t2))
                                        
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    Z = math.degrees(math.atan2(t3, t4))
    
    return X, Y, Z 


#global outfifo
#outfifo = open("/tmp/natnet-matlab-fifo",'w')
global sock
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

global oldstamp, olddelay
oldstamp = 0.0; olddelay = 0.0
global oldstamprcv, olddelayrcv
oldstamprcv  = 0.0; olddelayrcv = 0.0

if __debug__:
    def receiveRigidBodyList( rigidBodyList, stamp ):
        global outfifo
        global oldstamp, olddelay
        global oldstamprcv, olddelayrcv
        for (ac_id, pos, quat, valid) in rigidBodyList:
            if valid:

                stamprcv = time.time()

                oldstampdelay = (stamp - oldstamp)
                olddelayrcv = (stamprcv - oldstamprcv)

#                print(">")
#                print(oldstampdelay)
#                print(olddelayrcv)

                oldstamp = stamp
                oldstamprcv = stamprcv


                data_arr = [pos[j] for j in range(3)]
                data_arr[3:6] = quaternion_to_euler_angle(quat[0], quat[1], quat[2], quat[3])

                #msg = str(ac_id)+" "+" ".join(format(x,"+0.3f") for x in data_arr)+"\n"
                msg = str(ac_id)+" "+" "+format(stamp,"+0.5f")+" "+" ".join(format(x,"+0.3f") for x in data_arr)+"\n"
                print(msg)

                #outfifo.write(msg)
                #outfifo.flush()
                #sock.sendto(bytes(msg, "utf-8"),("127.0.0.1",5005))
                sock.sendto(bytes(msg, "utf-8"),("192.168.1.245",5005))


#                stamp = round(stamp,3)
#                period = round((stamp-oldstamp),3)
#                if(oldperiod != period):
#                    print(period)
#                oldstamp = stamp
#                oldperiod = period

                mainwindow.update(data_arr)

else:
    class Receiver(threading.Thread):
        def __init__(self, val):
            threading.Thread.__init__(self)
            self.val = val

        def run(self):
            while True:
                time.sleep(0.001)
                data_arr = [random.randint(1000*RangeParam[j][0], 1000*RangeParam[j][1])/1000
                        for j in range(NbParam)]
                self.val.update(data_arr)


def main():

    global mainwindow

    root = tk.Tk()
    mainwindow = Mainwindow(root)
    mainwindow.pack(side="top", fill="both", expand=True)

    if __debug__:
        natnet = NatNetClient(
            rigidBodyListListener=receiveRigidBodyList)
        natnet.run()
    else:
        rcv = Receiver(mainwindow)
        rcv.start() 

    root.mainloop()


# call main
if __name__ == '__main__':
    main()
