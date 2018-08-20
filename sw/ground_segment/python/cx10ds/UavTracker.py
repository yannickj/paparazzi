#!/usr/bin/python

# Control the position of a UAV along the camera axis

import sys
sys.path.insert(0, '/home/gautier/usr/lib/python2.7/dist-packages')
import cv2
import numpy as np
from PID import PID

from os import path, getenv
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python/")
from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage 

class UavDetector:
    # ###################################################################
    ## Constructor
    def __init__(self):
        # request to create a mask on first frame
        self.set_mask = True
        self.mask = None
        self.x = 0.
        self.y = 0.
        self.area = 0.
        self.threshold = 200
        

    # ###################################################################
    ## Process function with USB output
    def process(self, inframe, outframe=True):
        # Get the next camera image (may block until it is captured) and convert it to OpenCV GRAY:
        img = inframe
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Get image width, height, channels in pixels. Beware that if you change this module to get img as a grayscale
        # image, then you should change the line below to: "height, width = img.shape" otherwise numpy will throw. See
        # how it is done in the PythonOpenCv module of jevoisbase:
        self.height, self.width = gray.shape
        cv2.line(img,(int(self.width/2),int(self.height*0.1)), (int(self.width/2),int(self.height*0.9)), (0,0,255),1)
        cv2.line(img,(int(self.width*0.1),int(self.height/2)), (int(self.width*0.9),int(self.height/2)), (0,0,255),1)

        blur = cv2.GaussianBlur(gray,(5,5),0)
        ret,th = cv2.threshold(blur,self.threshold,255,cv2.THRESH_BINARY)
        if self.set_mask:
            self.mask = cv2.dilate(th, np.ones((10,10), np.uint8), iterations=1)
            self.mask = cv2.bitwise_not(self.mask)
            self.set_mask = False
        th_masked = cv2.bitwise_and(th, th, mask=self.mask)
        image, contours, hierarchy = cv2.findContours(th_masked,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            ## polygon contour
            #epsilon = 0.05*cv2.arcLength(cnt,True)
            #approx = cv2.approxPolyDP(cnt,epsilon,True)
            #pos = np.mean(approx, axis=0)
            #print("POS {} {}".format(pos[0][0]-self.width/2, -(pos[0][1]-self.height/2)));
            #cv2.drawContours(img, [approx], 0, (0,255,0), 3)

            ## bounding rectangle
            #x, y, w, h = cv2.boundingRect(cnt)
            #screenCnt = np.array([[[x, y]], [[x+w, y]], [[x+w, y+h]], [[x, y+h]]])
            ##print(x,y,w,h)
            #print("POS {} {} {}".format(x+w/2-self.width/2, -(y+h/2-self.height/2), h*w));
            #cv2.drawContours(img, [screenCnt], 0, (255, 255, 0), 2)

            ## rotated rectangle (min area)
            rect = cv2.minAreaRect(cnt)
            ((x, y), (w, h), _) = rect
            self.x = x-self.width/2
            self.y = -(y-self.height/2)
            self.area = w * h
            box = np.int0(cv2.boxPoints(rect))
            cv2.drawContours(img, [box], 0, (0,255,0), 3)
            print("POS {} {} {}".format(self.x, self.y, self.area))

        if outframe:
            #cv2.imshow('gray',gray)
            cv2.imshow('out',img)
         

    # #######################################################################
    ## Parse a serial command forwarded to us by the JeVois Engine, return a string
    # This function is optional and only needed if you want your module to handle custom commands. Delete if not needed.
    def parseSerial(self, str):
        print("parseserial received command [{}]".format(str))
        str_list = str.split(' ')
        if str == "set_mask":
            self.set_mask = True
            return("Mask set")
        elif str == "clear_mask":
            self.mask = np.zeros((self.height, self.width), np.uint8)
            self.mask = cv2.bitwise_not(self.mask)
            return("Mask cleared")
        elif len(str_list) == 2 and str_list[0] == "set_thres" and str_list[1].isdigit():
            self.threshold = max(0, min(255, int(str_list[1])))
            return("Threshold set")
        return "ERR: Unsupported command"
    
    # ##########################################################################
    ## Return a string that describes the custom commands we support, for the JeVois help message
    # This function is optional and only needed if you want your module to handle custom commands. Delete if not needed.
    def supportedCommands(self):
        # use \n seperator if your module supports several commands
        return "set_mask - hide visible objects from the scene\nclear_mask - clear all mask (show all objects)"

class Controller():
    def __init__(self):
        self.pid_lat = PID(P=1.0,D=0.5,I=0.1)
        self.pid_vert = PID(P=0.5,D=0.5,I=0.)
        self.speed = 128 ## openloop forward control

        # Start IVY interface
        self._interface = IvyMessagesInterface("UavTracker Control")
        self.msg = PprzMessage("ground", "JOYSTICK")


    def __del__(self):
        self.stop()

    def stop(self):
        # Stop IVY interface
        if self._interface is not None:
            self._interface.shutdown()


    def set_speed(self, speed):
        self.speed = speed

    def run(self, lat, vert, dist):
        self.pid_lat.update(lat)
        self.pid_vert.update(vert)
        self.msg['axis1'] = int(self.pid_lat.output) #-int(lat) # roll
        self.msg['axis2'] = self.speed # pitch
        self.msg['axis3'] = 127 # yaw
        self.msg['axis4'] = int(abs(self.pid_vert.output)) #int(abs(vert)) # throttle_incr
        if self.pid_vert.output > 0.1:
            self.msg['button1'] = 2 # direction
        elif self.pid_vert.output < -0.1:
            self.msg['button1'] = 0 # direction
        else:
            self.msg['button1'] = 1 # direction
        self.msg['button2'] = 0 # land
        self.msg['button3'] = 0 # takeoff
        self.msg['button4'] = 0 # idle
        self._interface.send(self.msg)


if __name__ == '__main__':

    detector = UavDetector()
    ctrl = Controller()


    cv2.namedWindow('out')
    # create trackbars for color change
    cv2.createTrackbar('P lat','out',int(ctrl.pid_lat.Kp*100),1000,lambda x: ctrl.pid_lat.setKp(x/100.))
    cv2.createTrackbar('I lat','out',int(ctrl.pid_lat.Ki*100),1000,lambda x: ctrl.pid_lat.setKi(x/100.))
    cv2.createTrackbar('D lat','out',int(ctrl.pid_lat.Kd*100),1000,lambda x: ctrl.pid_lat.setKd(x/100.))
    cv2.createTrackbar('P vert','out',int(ctrl.pid_vert.Kp*100),1000,lambda x: ctrl.pid_vert.setKp(x/100.))
    cv2.createTrackbar('I vert','out',int(ctrl.pid_vert.Ki*100),1000,lambda x: ctrl.pid_vert.setKi(x/100.))
    cv2.createTrackbar('D vert','out',int(ctrl.pid_vert.Kd*100),1000,lambda x: ctrl.pid_vert.setKd(x/100.))
    cv2.createTrackbar('Speed','out',ctrl.speed,255,ctrl.set_speed)
    cv2.createTrackbar('Thres','out',detector.threshold,255,lambda x: detector.parseSerial("set_thres {}".format(x)))


    cap = cv2.VideoCapture(0)

    while (True):
        # Capture frame-by-frame
        ret, frame = cap.read()

        detector.process(frame)
        ctrl.run(detector.x, detector.y, detector.area)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    ctrl.stop()
    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

