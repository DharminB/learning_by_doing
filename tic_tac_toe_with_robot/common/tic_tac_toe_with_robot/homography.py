#! /usr/bin/env python

from __future__ import print_function

import cv2
import time
import numpy as np

ARUCO_DICT = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

def get_aruco_marker_points(img):
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(img, ARUCO_DICT)
    if len(ids) == 0:
        return None
    centers = np.mean(corners, axis=2)
    return np.squeeze(ids), np.squeeze(centers)

def main():
    video_feed = cv2.VideoCapture('/home/dharmin/Videos/static.avi')
    ideal_ids = np.array([11, 3, 7, 10])
    while video_feed.isOpened():
        time.sleep(0.1)
        ret, frame = video_feed.read()
        if not ret:
            break
        frame = cv2.flip(frame, flipCode=1)
        cv2.imshow('frame', frame)

        ans = get_aruco_marker_points(frame)
        if ans is None or not np.array_equal(np.sort(ans[0]), np.sort(ideal_ids)):
            continue
        ids, points = ans
        # arrange the points to ideal_ids
        indexes = np.where(ideal_ids.reshape(ideal_ids.size, 1) == ids)[1]
        points = points[indexes]

        ris = 400 # resulting image size
        dst_pts = np.array([[0, ris-1], [ris-1, ris-1], [ris-1, 0], [0, 0]])

        h, status = cv2.findHomography(points, dst_pts)
        sq_img = cv2.warpPerspective(frame, h, (ris, ris))
        cv2.imshow('sq_img', sq_img)

        crop_start_perc, crop_end_perc = 0.10, 0.90
        cropped_sq_img = sq_img[int(ris*crop_start_perc):int(ris*crop_end_perc),
                                int(ris*crop_start_perc):int(ris*crop_end_perc)]
        cv2.imshow('cropped_sq_img', cropped_sq_img)

        tile_start_perc, tile_end_perc = 0.15, 0.85
        tile_size = int(cropped_sq_img.shape[0]/3)
        tiles = [[0,0,0], [0,0,0], [0,0,0]]
        for i in range(3):
            for j in range(3):
                temp = cropped_sq_img[i*tile_size:(i+1)*tile_size, j*tile_size:(j+1)*tile_size]
                tiles[i][j] = temp[int(tile_size*tile_start_perc):int(tile_size*tile_end_perc),
                                   int(tile_size*tile_start_perc):int(tile_size*tile_end_perc)]
                cv2.imshow('tile_'+str(i)+'_'+str(j), tiles[i][j])

        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break
        if cv2.waitKey(0):
            break
    video_feed.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
