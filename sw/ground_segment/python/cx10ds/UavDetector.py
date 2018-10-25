#!/usr/bin/python

# Control the position of a UAV along the camera axis

import sys
sys.path.insert(0, '/home/gautier/usr/lib/python2.7/dist-packages')
import cv2
import numpy as np
#from math import sqrt, atan

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
        self.heading = 0.
        self.threshold = 150
        self.new_data = False
        self.bk = None
        self.center = None
        self.filter = True
        self.filter_last = None
        self.filter_max = 500 # jump prevention in pixels
        

    # ###################################################################
    ## Process function with USB output
    def process(self, inframe, outframe=False):
        # Get the next camera image (may block until it is captured) and convert it to OpenCV GRAY:
        if outframe:
            img = cv2.cvtColor(inframe, cv2.COLOR_GRAY2BGR)
        #gray = cv2.cvtColor(inframe, cv2.COLOR_BGR2GRAY)
        gray = inframe

        # Get image width, height, channels in pixels. Beware that if you change this module to get img as a grayscale
        # image, then you should change the line below to: "height, width = img.shape" otherwise numpy will throw.
        self.height, self.width = gray.shape
        self.center = (self.width/2, self.height/2)
        if outframe:
            cv2.line(img,(int(self.center[0]),int(self.height*0.1)), (int(self.center[0]),int(self.height*0.9)), (0,0,255),1)
            cv2.line(img,(int(self.width*0.1),int(self.center[1])), (int(self.width*0.9),int(self.center[1])), (0,0,255),1)

        blur = cv2.GaussianBlur(gray,(5,5),0)
        if self.set_mask:
            print("set mask")
            _,th = cv2.threshold(blur,self.threshold/2,255,cv2.THRESH_BINARY)
            self.mask = cv2.dilate(th, np.ones((40,40), np.uint8), iterations=1)
            if outframe:
                # build mask image
                self.bk = np.zeros((self.height,self.width,3),np.uint8)
                self.bk[:] = (0,0,255)
                self.bk = cv2.bitwise_and(self.bk, self.bk, mask=self.mask)
            self.mask = cv2.bitwise_not(self.mask) # invert mask
            self.set_mask = False
        else:
            _,th = cv2.threshold(blur,self.threshold,255,cv2.THRESH_BINARY)
        th_masked = cv2.bitwise_and(th, th, mask=self.mask)
        _, contours, _ = cv2.findContours(th_masked,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        
        # call correct feature extraction function
        self.extract_rect(contours, img, outframe)
        #self.extract_triangle(contours, img, outframe)
        #self.extract_line(contours, img, outframe)

        if outframe:
            #cv2.imshow('gray',gray)
            if self.bk is not None:
                fg = cv2.bitwise_and(img, img, mask=self.mask)
                img = cv2.bitwise_or(fg, self.bk)
            scale_percent = 60 # percent of original size
            width = int(img.shape[1] * scale_percent / 100)
            height = int(img.shape[0] * scale_percent / 100)
            dim = (width, height)
# resize image
            resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA) 
            cv2.imshow('out',resized)

    def extract_rect(self, contours, img, outframe):
        def dist(c1, c2):
            (x1,y1) = c1
            (x2,y2) = c2
            return abs(x1-x2) + abs(y1-y2)

        if len(contours) == 0:
            self.new_data = False
            return
        elif len(contours) == 1:
            rect = cv2.minAreaRect(contours[0])
            if self.filter and self.filter_last is not None:
                (p, _, _) = rect
                d = dist(self.filter_last, p)
                if d < self.filter_max:
                    (self.filter_last, _, _) = rect
                else:
                    self.new_data = False
                    return # this might be a jump
            else:
                (self.filter_last, _, _) = rect
        else:
            if self.filter and self.filter_last is not None:
                # try to find the best match if more than one object
                best_dist = 9999
                rect = None
                for cnt in contours:
                    r = cv2.minAreaRect(cnt)
                    (p, _, _) = r
                    d = dist(self.filter_last, p)
                    if d < best_dist:
                        rect = r
                        best_dist = d
                if best_dist < self.filter_max:
                    (self.filter_last, _, _) = rect
                else:
                    self.new_data = False
                    return # this might be a jump
            else:
                self.new_data = False
                for cnt in contours:
                    rect = cv2.minAreaRect(cnt)
                    box = np.int0(cv2.boxPoints(rect))
                    cv2.drawContours(img, [box], 0, (0,255,0), 3)
                return
        
        ## rotated rectangle (min area)
        ((x, y), (w, h), _) = rect
        #if abs(w - h) > 10: # not square
        #    continue
        self.x = x-self.center[0]
        self.y = -(y-self.center[1])
        self.area = w * h
        self.heading = 0.
        self.new_data = True
        if outframe:
            box = np.int0(cv2.boxPoints(rect))
            cv2.drawContours(img, [box], 0, (0,255,0), 3)

    def extract_line(self, contours, img, outframe):
        if len(contours) != 2:
            self.new_data = False
            return

        centers = []
        for cnt in contours:
            x,y,w,h = cv2.boundingRect(cnt)
            centers.append((x+w/2.,y+h/2.))
        centers = sorted(centers, key = lambda c: c[0]) # sort centers by lateral coord
        xi = (centers[0][0] + centers[1][0]) / 2.
        yi = (centers[0][1] + centers[1][1]) / 2.
        self.x = xi - self.center[0]
        self.y = -(yi - self.center[1])
        self.area = self.compute_dist2(centers[0], centers[1])
        self.heading = 0.
        self.new_data = True
        if outframe:
            cv2.line(img,(int(centers[0][0]),int(centers[0][1])),(int(centers[1][0]),int(centers[1][1])), (0,255,0),2)
            cv2.circle(img,(int(xi),int(yi)), 10, (0,255,0), 1)

    def extract_triangle(self, contours, img, outframe):
        if len(contours) != 3:
            self.new_data = False
            return

        centers = []
        for cnt in contours:
            x,y,w,h = cv2.boundingRect(cnt)
            centers.append((x+w/2.,y+h/2.))
        centers = sorted(centers, key = lambda c: c[0]) # sort centers by lateral coord
        xi = (centers[0][0] + centers[2][0]) / 2.
        yi = (centers[0][1] + centers[2][1]) / 2.
        self.x = xi - self.center[0]
        self.y = -(yi - self.center[1])
        self.area = self.compute_dist2(centers[0], centers[2])
        dx = centers[2][0] - centers[0][0] 
        if dx > 5.:
            self.heading = centers[1][0] - dx/2.
        else:
            self.heading = 0.
        self.new_data = True
        if outframe:
            cv2.line(img,(int(centers[0][0]),int(centers[0][1])),(int(centers[2][0]),int(centers[2][1])), (0,255,0),2)
            cv2.circle(img,(int(xi),int(yi)), 10, (0,255,0), 1)
            cv2.circle(img,(int(centers[1][0]),int(centers[1][1])),10,(0,255,255),1)

    def compute_dist2(self, p1, p2):
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        return dx*dx + dy*dy

    def setmask(self):
        self.set_mask = True
        self.filter_last = None
        print("Mask set")

    def clearmask(self):
        self.mask = np.zeros((self.height, self.width), np.uint8)
        self.mask = cv2.bitwise_not(self.mask)
        self.bk = np.zeros((self.height,self.width,3),np.uint8)
        self.filter_last = None
        print("Mask cleared")

    def set_thres(self, val):
        self.threshold = max(0, min(255, int(val)))
        print("Threshold set: {}".format(self.threshold))


