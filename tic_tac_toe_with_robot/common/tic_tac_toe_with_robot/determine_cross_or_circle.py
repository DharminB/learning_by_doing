#! /usr/bin/env python

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

if __name__ == "__main__":
    # img = cv2.imread('cross.jpg', cv2.IMREAD_GRAYSCALE)
    # print(is_tile_cross(img, (250, 150, 460, 350), 30))
    # print(is_tile_cross('circle.jpg', (250, 150, 460, 350), 30))

    cap = cv2.VideoCapture(1)
    while True:
        time.sleep(0.2)
        ret, frame = cap.read()
        img = cv2.flip(frame, flipCode=-1)
        cv2.imshow('frame', img)
        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # print(is_tile_cross(gray, (200, 100, 400, 330), 30))
        # print(gray.shape)
        # cv2.imshow('gray', gray)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()
