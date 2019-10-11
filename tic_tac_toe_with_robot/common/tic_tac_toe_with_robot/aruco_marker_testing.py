#! /usr/bin/env python

from __future__ import print_function

import os
import cv2
import yaml
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

CAMERA_MAT = None
DIST_COEFF = None
MARKER_SIZE = 0.04
ARUCO_DICT = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

class ArucoMarkerTesting(object):
    def __init__(self, calibration_file_path):
        self.video_feed = cv2.VideoCapture(1)
        with open(calibration_file_path, 'r') as file_obj:
            data = yaml.safe_load(file_obj)
            self.camera_mat = np.array(data['camera_matrix'])
            self.dist_coeff = np.array(data['dist_coeff'])
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.marker_size = 0.04

    def __del__(self):
        self.video_feed.release()
        cv2.destroyAllWindows()

    def get_aruco_marker_and_axis(self, visualise=False):
        for i in range(4):
            self.video_feed.grab()
        ret, img = self.video_feed.read()
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(img, self.aruco_dict)
        if ids is not None:
            rot, trans, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_mat, self.dist_coeff)
            if visualise:
                self.visualise_aruco_marker_and_axis(img, ids, corners, rot, trans)
            return ids, corners, rot, trans
        else:
            return None

    def visualise_aruco_marker_and_axis(self, img, ids, corners, rot, trans):
        frame_axis = cv2.aruco.drawDetectedMarkers(img.copy(), corners, ids)
        height, width, _ = img.shape
        for marker_id in range(ids.shape[0]):
            frame_axis = cv2.aruco.drawAxis(frame_axis, self.camera_mat, self.dist_coeff, rot[marker_id], trans[marker_id], 0.02)
        cv2.imshow('frame_axis', frame_axis)
        cv2.waitKey(1)


def get_path_from_name(file_name):
    current_dir = os.path.dirname(os.path.abspath(__file__))
    main_dir = os.path.dirname(os.path.dirname(current_dir))
    config_file = os.path.join(main_dir, 'ros/config/' + file_name)
    return config_file

def main():
    calibration_file_path = get_path_from_name('calibration.yaml')
    amt = ArucoMarkerTesting(calibration_file_path)
    while True:
        time.sleep(0.1)
        result = amt.get_aruco_marker_and_axis(visualise=True)
    video_feed.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
