#! /usr/bin/env python
from __future__ import print_function

import cv2
import time
import numpy as np

def show_image(img):
    cv2.imshow('image', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def is_tile_cross(img, roi, centre_width):
    """Determine if `roi` (region of interest) of `img_name` is a X or a O

    :img_name: cv2 image obj
    :roi: sequence(int, int, int, int)
    :centre_width: int
    :return: bool
    """
    cropped_piece = img[roi[0]:roi[2], roi[1]:roi[3]]
    # show_image(cropped_piece)
    gaus_mask = cv2.adaptiveThreshold(cropped_piece, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                      cv2.THRESH_BINARY, 125, 1)
    # show_image(gaus_mask)
    cv2.imshow('frame', gaus_mask)
    height, width = gaus_mask.shape
    gaus_crop = gaus_mask[height/2-centre_width:height/2+centre_width,
                          width/2-centre_width:width/2+centre_width]
    # show_image(gaus_crop)
    return np.any(gaus_crop == 0)

def get_comp_img(labels):
    # Map component labels to hue val
    label_hue = np.uint8(179*labels/np.max(labels))
    blank_ch = 255*np.ones_like(label_hue)
    labeled_img = cv2.merge([label_hue, blank_ch, blank_ch])

    # cvt to BGR for display
    labeled_img = cv2.cvtColor(labeled_img, cv2.COLOR_HSV2BGR)

    # set bg label to black
    labeled_img[label_hue==0] = 0
    return labeled_img

def connected_comp(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    bw_img = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                      cv2.THRESH_BINARY_INV, 125, 10)
    cv2.imshow('bw_img', bw_img)
    ret, labels, stats, centroids = cv2.connectedComponentsWithStats(bw_img)
    for i in range(labels.max()):
        if not (60 < stats[i][-1] < 600):
            labels[labels==i] = 0
    print("Found", np.unique(labels).shape[0], "components")
    labeled_img = get_comp_img(labels)
    for index in range(np.unique(labels).shape[0]):
        indices = np.argwhere(labels==np.unique(labels)[index])
        if indices.shape[0] < 10:
            continue
        ellipse = cv2.fitEllipse(np.flip(indices, axis=1))
        # print(ellipse)
        if abs(ellipse[1][0] - ellipse[1][1]) < 4 and 20 < ellipse[1][0] < 30:
            rms_error = 0
            radius = (ellipse[1][0] + ellipse[1][0])/4
            for ind in np.flip(indices, axis=1):
                rms_error += abs(np.linalg.norm(ind - ellipse[0]) - radius)
            avg_rms_error = rms_error/indices.shape[0]
            # print(avg_rms_error)
            if avg_rms_error < 1.7:
                cv2.circle(labeled_img, tuple(map(int, ellipse[0])), 5, (0, 0, 255), -1)
                labeled_img = cv2.ellipse(labeled_img, ellipse, (255, 255, 255), 2)
    cv2.imshow('labeled_img', labeled_img)

def contours(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    bw_img = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                      cv2.THRESH_BINARY_INV, 125, 10)
    cv2.imshow('bw_img', bw_img)
    _, contours, hierarchy = cv2.findContours(bw_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    indexes = filter(lambda x: 50 <cv2.contourArea(contours[x]) < 600, range(len(contours)))
    contour_img = img.copy()
    # contour_img = cv2.drawContours(contour_img, contours, -1, (0, 255, 0), 1)
    # print(len(contours))
    for cnt in contours[:]:
        if cnt.shape[0] < 10:
            continue
        ellipse = cv2.fitEllipse(cnt)
        # print(ellipse)
        if abs(ellipse[1][0] - ellipse[1][1]) < 4 and 15 < ellipse[1][0] < 30:
            rms_error = 0
            radius = (ellipse[1][0] + ellipse[1][0])/4
            for pt in cnt:
                rms_error += abs(np.linalg.norm(pt - ellipse[0]) - radius)
            avg_rms_error = rms_error/cnt.shape[0]
            # print(avg_rms_error)
            if avg_rms_error < 2:
                cv2.circle(contour_img, tuple(map(int, ellipse[0])), 5, (0, 0, 255), -1)
                contour_img = cv2.ellipse(contour_img, ellipse, (255, 255, 255), 3)
        # else:
        #     contour_img = cv2.ellipse(contour_img, ellipse, (0, 255, 255), 1)

    # M = cv2.moments(cnt)
    # centroid = (int(M['m10']/M['m00']), int(M['m01']/M['m00']))
    # contour_img = cv2.drawContours(img, [cnt], 0, (0, 0, 255), 1)
    # for i in indexes:
    #     contour_img = cv2.drawContours(contour_img, contours, i, (0, 0, 255), -1)
    # cv2.circle(contour_img, centroid, 4, (0, 255, 0), -1)
    # area = cv2.contourArea(cnt)
    # perimeter = cv2.arcLength(cnt, True)
    cv2.imshow('contour_img', contour_img)

if __name__ == "__main__":
    # video_feed = cv2.VideoCapture(1)
    video_feed = cv2.VideoCapture('/home/dharmin/Downloads/01.avi')
    while video_feed.isOpened():
        time.sleep(0.2)
        ret, frame = video_feed.read()
        # print(ret)
        if not ret:
            break
        frame = cv2.flip(frame, flipCode=1)
        # cv2.imshow('frame', frame)
        contours(frame)
        # connected_comp(frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        # if cv2.waitKey(0)
        #     print("waiting")
        #     if cv2.waitKey(0) & 0xFF == ord('q'):
        #         break
    video_feed.release()
    cv2.destroyAllWindows()
