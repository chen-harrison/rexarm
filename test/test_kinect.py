#!/usr/bin/python3
"""!
Test the kinect

TODO: Use this file and modify as you see fit to test kinect
"""
import os
script_path = os.path.dirname(os.path.realpath(__file__))
os.sys.path.append(script_path + '/../')
import sys
import cv2
import numpy as np
from kinect import *

font = cv2.FONT_HERSHEY_SIMPLEX

def mouse_callback(event, x, y, flags, param):
    img = video_frame
    hsv = cv2.cvtColor(kinect.VideoFrame, cv2.COLOR_RGB2HSV)
    r = img[y][x][2]
    g = img[y][x][1]
    b = img[y][x][0]
    h = hsv[y][x][0]
    s = hsv[y][x][1]
    v = hsv[y][x][2]
    output_rgb = "R:{}, G:{}, B:{} ".format(r, g, b)
    output_hsv = "H:{}, S:{}, V:{}".format(h, s, v)
    tmp = video_frame.copy()
    cv2.putText(tmp,output_rgb, (10, 20), font, 0.5, (0, 0, 0))
    cv2.putText(tmp,output_hsv, (10, 40), font, 0.5, (0, 0, 0))
    if event == cv2.EVENT_LBUTTONDOWN:
        print("RGB Clicked point: (",x,',',y,")")
    # cv2.imshow('Video', tmp)

def mouse_callback_depth(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print("Depth Clicked point: (",x,',',y,")")

kinect = Kinect()

cv2.namedWindow('Depth')
cv2.namedWindow('Video')
print('Press ESC in window to stop')

rgb_cal_coords = np.array([[153,430],[157,84],[500,86],[501,431],
                           [398,356],[425,227],[157,227], [238,390]])
depth_cal_coords = np.array([[172,422], [172,46], [541,46], [547,420],
                             [435,343], [470,200], [170,200], [456,227]])
kinect.depth2rgb_affine = kinect.getAffineTransform(depth_cal_coords, rgb_cal_coords)
print(kinect.depth2rgb_affine)

while True:
    kinect.captureVideoFrame()
    kinect.captureDepthFrame()
    kinect.ColorizeDepthFrame()
    depth_frame = cv2.cvtColor(kinect.DepthFrameRGB, cv2.COLOR_RGB2BGR)
    video_frame = cv2.cvtColor(kinect.VideoFrame, cv2.COLOR_RGB2BGR)

    #video_frame = kinect.VideoFrame
    #video_frame =  cv2.cvtColor(kinect.VideoFrame, cv2.COLOR_RGB2HSV)
    #hsv_frame = cv2.cvtColor(kinect.VideoFrame, cv2.COLOR_RGB2HSV)
    #output_frame = cv2.inRange(hsv_frame,(110,100,100), (120,255,255))
    #kernel = np.ones((4,4),np.uint8)
    #output_frame = cv2.erode(output_frame, kernel)
    #output_frame = cv2.dilate(output_frame, kernel, iterations=2)
    #depth_frame = cv2.warpAffine(depth_frame, kinect.depth2rgb_affine, (640,480))
    
    cv2.imshow('Depth', depth_frame)
    cv2.imshow('Video', video_frame)
    cv2.setMouseCallback("Video",mouse_callback)
    cv2.setMouseCallback("Depth", mouse_callback_depth)
    #cv2.imshow('Output', output_frame)
    k = cv2.waitKey(10)
    if(k == 27): # 'Esc' key
        break
    elif(k == 97): # 'a' key
        kinect.toggleExposure(True)
    elif(k==120): # 'x' key
        kinect.toggleExposure(False)
    elif(k==112): # 'p' key
        cv2.imwrite(script_path + "/../data/rgb_image.png", cv2.cvtColor(kinect.VideoFrame, cv2.COLOR_RGB2BGR))
        depth = kinect.DepthFrameRaw.astype(np.uint16) * 256
        cv2.imwrite(script_path + "/../data/raw_depth.png", depth)
        print("picture taken\n")
    elif(k != -1):
        pass
        #print("Options: \n'Esc' to Quit, \n'a' to turn on autogain \n'x' to turn off autogain \n'p' to take an image")
