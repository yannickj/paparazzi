#!/usr/bin/python

# Control the position of a UAV along the camera axis

import sys
sys.path.insert(0, '/home/gautier/usr/lib/python2.7/dist-packages')
import cv2
import numpy as np

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
        self.threshold = 150
        self.new_data = False
        

    # ###################################################################
    ## Process function with USB output
    def process(self, inframe, outframe=False):
        # Get the next camera image (may block until it is captured) and convert it to OpenCV GRAY:
        img = inframe
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Get image width, height, channels in pixels. Beware that if you change this module to get img as a grayscale
        # image, then you should change the line below to: "height, width = img.shape" otherwise numpy will throw. See
        # how it is done in the PythonOpenCv module of jevoisbase:
        self.height, self.width = gray.shape
        if outframe:
            cv2.line(img,(int(self.width/2),int(self.height*0.1)), (int(self.width/2),int(self.height*0.9)), (0,0,255),1)
            cv2.line(img,(int(self.width*0.1),int(self.height/2)), (int(self.width*0.9),int(self.height/2)), (0,0,255),1)

        blur = cv2.GaussianBlur(gray,(5,5),0)
        ret,th = cv2.threshold(blur,self.threshold,255,cv2.THRESH_BINARY)
        if self.set_mask:
            print("set mask")
            self.mask = cv2.dilate(th, np.ones((40,40), np.uint8), iterations=1)
            self.mask = cv2.bitwise_not(self.mask)
            self.set_mask = False
        th_masked = cv2.bitwise_and(th, th, mask=self.mask)
        image, contours, hierarchy = cv2.findContours(th_masked,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            ## rotated rectangle (min area)
            rect = cv2.minAreaRect(cnt)
            ((x, y), (w, h), _) = rect
            self.x = x-self.width/2
            self.y = -(y-self.height/2)
            self.area = w * h
            self.new_data = True
            if outframe:
                box = np.int0(cv2.boxPoints(rect))
                cv2.drawContours(img, [box], 0, (0,255,0), 3)

        if outframe:
            #cv2.imshow('gray',gray)
            scale_percent = 60 # percent of original size
            width = int(img.shape[1] * scale_percent / 100)
            height = int(img.shape[0] * scale_percent / 100)
            dim = (width, height)
# resize image
            resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA) 
            cv2.imshow('out',resized)
         

    def set_mask(self):
        self.set_mask = True
        print("Mask set")

    def clear_mask(self):
        self.mask = np.zeros((self.height, self.width), np.uint8)
        self.mask = cv2.bitwise_not(self.mask)
        print("Mask cleared")

    def set_thres(self, val):
        self.threshold = max(0, min(255, int(val)))
        print("Threshold set: {}".format(self.threshold))


