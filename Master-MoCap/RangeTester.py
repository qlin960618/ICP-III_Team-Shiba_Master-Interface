#!/usr/bin/env python
# -*- coding: utf-8 -*-

# USAGE: You need to specify a filter and "only one" image source
#
# (python) range-detector --filter RGB --image /path/to/image.png
# or
# (python) range-detector --filter HSV --webcam

import cv2
import argparse
from operator import xor

DEFAULT_greenLimit = [[54, 217, 16], [112, 255, 84]]
DEFAULT_redLimit = [[156, 154, 80], [198, 255, 176]]


def callback(value):
    pass


def get_trackbar_values():
    values = []

    for i in ["MIN", "MAX"]:
        for j in 'HSV':
            v = cv2.getTrackbarPos("%s_%s" % (j, i), "Trackbars")
            values.append(v)

    return values

def setup_trackbars(limit):
    cv2.namedWindow("Trackbars", 0)

    list_name=[]
    for i in ["MIN", "MAX"]:
        # v = 0 if i == "MIN" else 255
        for j in 'HSV':
            list_name=list_name+["%s_%s" % (j, i)]
    for i in range(len(list_name)):
        cv2.createTrackbar(list_name[i], "Trackbars", limit[i], 255, callback)
        # cv2.setTrackbarPos("%s_%s" % (j, i), "Trackbars")


def get_arguments():
    ap = argparse.ArgumentParser()
    ap.add_argument('-f', '--filter', required=True,
                    help='Range filter. [G]REEN or [R]ED')
    ap.add_argument('-w', '--webcam', required=True,
                    help='Use webcam #')
    args = vars(ap.parse_args())

    return args


def main():
    args = get_arguments()

    range_filter = args['filter'].upper()

    camera = cv2.VideoCapture(int(args['webcam']))
    if range_filter=='G':
        hsv_lower = tuple(DEFAULT_greenLimit[0])
        hsv_upper = tuple(DEFAULT_greenLimit[1])
    elif range_filter=='R':
        hsv_lower = tuple(DEFAULT_redLimit[0])
        hsv_upper = tuple(DEFAULT_redLimit[1])
    hsvlim=(*hsv_lower, *hsv_upper)

    setup_trackbars(hsvlim)

    while True:

        ret, image = camera.read()

        if not ret:
            break

        frame_to_thresh = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        v1_min, v2_min, v3_min, v1_max, v2_max, v3_max = get_trackbar_values()
        thresh = cv2.inRange(frame_to_thresh, (v1_min, v2_min, v3_min), (v1_max, v2_max, v3_max))
        thresh = cv2.erode(thresh, None, iterations=2)
        thresh = cv2.dilate(thresh, None, iterations=2)

        cv2.imshow("Original", image)
        cv2.imshow("Thresh", thresh)

        if cv2.waitKey(1) & 0xFF is ord('q'):
            break


if __name__ == '__main__':
    main()
