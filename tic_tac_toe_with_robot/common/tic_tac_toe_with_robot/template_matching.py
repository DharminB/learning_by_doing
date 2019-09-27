#! /usr/bin/env python

import cv2
import time
import numpy as np

def match_template(img, template):
    height, width = template.shape
    res = cv2.matchTemplate(img, template, cv2.TM_CCOEFF_NORMED)
    threshold = 0.5
    loc = np.where(res >= threshold)
    for pt in zip(*loc[::-1]):
        cv2.rectangle(img, pt, (pt[0]+w, pt[1]+h), 255, 20)
    cv2.imshow('detected', img)

if __name__ == "__main__":
    template = cv2.imread('template.png', cv2.IMREAD_GRAYSCALE)

    cap = cv2.VideoCapture(1)
    while True:
        time.sleep(0.2)
        ret, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        match_template(gray, template)
        cv2.imshow('gray', gray)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()
