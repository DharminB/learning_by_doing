#! /usr/bin/env python

from __future__ import print_function

import cv2
import time
import numpy as np

ARUCO_DICT = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

def get_aruco_marker_points(img):
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(img, ARUCO_DICT)
    if ids is None:
        return None
    centers = np.mean(corners, axis=2)
    return np.squeeze(ids), np.squeeze(centers)

def get_tiles(img, **kwargs):
    """Get 9 tiles (3 x 3 list of cv2.Image) from a raw image.
    Assumption: The tic tac toe board is surrounded by aruco markers at each corner.

    :img: cv2.Image

    key word arguments
        :ideal_ids: np.Array of shape 4x1 int (order of aruco marker ids from 
                                               image bottom left counter clockwise)
        :ris: int (resulting image size for homography matching)
        :crop_perc: float between 0 and 1 (crop percentage for whole homography image)
        :tile_crop_perc: float between 0 and 1 (crop percentage for tile images)
        :debug: bool (show cv2 debug image or not)

    """
    debug = kwargs.get('debug', False)
    ideal_ids = kwargs.get('ideal_ids', [])
    ans = get_aruco_marker_points(img)
    if ans is None or not np.array_equal(np.sort(ans[0]), np.sort(ideal_ids)):
        return None
    ids, points = ans
    # arrange the points to ideal_ids
    indexes = np.where(ideal_ids.reshape(ideal_ids.size, 1) == ids)[1]
    points = points[indexes]

    ris = kwargs.get('ris', 400) # resulting image size
    dst_pts = np.array([[0, ris-1], [ris-1, ris-1], [ris-1, 0], [0, 0]])

    h, status = cv2.findHomography(points, dst_pts)
    sq_img = cv2.warpPerspective(img, h, (ris, ris))

    crop_perc = kwargs.get('crop_perc', 0.10)
    cropped_sq_img = sq_img[int(ris*crop_perc):int(ris*(1.0-crop_perc)),
                            int(ris*crop_perc):int(ris*(1.0-crop_perc))]
    if debug:
        cv2.imshow('sq_img', sq_img)
        cv2.imshow('cropped_sq_img', cropped_sq_img)

    tile_crop_perc = kwargs.get('tile_crop_perc', 0.15)
    tile_size = int(cropped_sq_img.shape[0]/3)
    tiles = [[0,0,0], [0,0,0], [0,0,0]]
    for i in range(3):
        for j in range(3):
            temp = cropped_sq_img[i*tile_size:(i+1)*tile_size, j*tile_size:(j+1)*tile_size]
            tiles[i][j] = temp[int(tile_size*tile_crop_perc):int(tile_size*(1.0-tile_crop_perc)),
                               int(tile_size*tile_crop_perc):int(tile_size*(1.0-tile_crop_perc))]
    return tiles

def main(video=True):
    if video:
        video_feed = cv2.VideoCapture('/home/dharmin/Videos/static.avi')
    else:
        video_feed = cv2.VideoCapture(1)
    ideal_ids = np.array([11, 3, 7, 10])
    while video_feed.isOpened():
        # time.sleep(0.1)
        ret, frame = video_feed.read()
        if not ret:
            break
        if video:
            frame = cv2.flip(frame, flipCode=1)
        cv2.imshow('frame', frame)

        tiles = get_tiles(frame, ideal_ids=ideal_ids)
        if tiles is None:
            continue
        for i in range(1):
            for j in range(1):
                cv2.imshow('tile_'+str(i)+'_'+str(j), tiles[i][j])

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        # if cv2.waitKey(0):
        #     break
    video_feed.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main(False)
