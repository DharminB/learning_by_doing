#! /usr/bin/env python

from __future__ import print_function

import cv2
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

if __name__ == "__main__":
    # template = cv2.imread('template.png', cv2.IMREAD_GRAYSCALE)

    video_feed = cv2.VideoCapture(1)
    for i in range(4):
        video_feed.grab()
    ret, frame = video_feed.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    params = cv2.aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)
    print(corners)
    print(ids)
    frame_markers = cv2.aruco.drawDetectedMarkers(frame.copy(), corners, ids)
    cv2.imshow('frame_markers', frame_markers)
    # cv2.imshow('gray', gray)
    cv2.waitKey(0)
    video_feed.release()
    cv2.destroyAllWindows()
    # # aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)
    # print(dir(aruco_dict))
    # print(aruco_dict.markerSize)
    # for i in range(50):
    #     img = cv2.aruco.drawMarker(aruco_dict,i, 300)
    #     cv2.imshow('gray', img)
    #     cv2.waitKey(0)
