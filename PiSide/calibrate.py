import sys
import os
import cv2
import numpy as np

import config, image

f = open('calibration.txt', 'r')
print(f.readlines())
f.close()

variables = []
coordinates = []
img = config.initial_img.copy()
marks_updated = False

scaleY = 0.7
scaleX = 1.0


def readcalibration():
    global img
    img = config.initial_img.copy()
    global variables
    variables = []
    with open('calibration.txt', 'r') as f:
        line_number = 0
        for line in f:
            line = float(line.strip())
            variables.append(line)
            line_number += 1
        print(variables)
    config.white_piece_low = variables[0:3]
    config.white_piece_high = variables[3:6]
    config.chess_coordinates = variables[6:14]
    f.close()

def mouse_callback(event, x, y, flags, param):
    global coordinates
    global marks_updated

    if event == cv2.EVENT_LBUTTONDOWN:
        # Display on user image
        cv2.circle(img, (x, y), 10, [255, 0, 0], -1)
        marks_updated = True
        coordinates.append([x, y])

def calibrate_transform():
    global marks_updated
    global coordinates
    coordinates = []
    clicks = 0
    cv2.namedWindow('image')
    cv2.setMouseCallback('image', mouse_callback)
    print(img.shape)
    while clicks < 4:
        cv2.imshow('image', img)

        # close everything if Esc is pressed
        k = cv2.waitKey(1)

        if k == 27:
            break

        if marks_updated:
            clicks += 1
            marks_updated = False

    print('coordinates are: {}'.format(coordinates))
    config.chess_coordinates[0:2] = coordinates[0][:]
    config.chess_coordinates[2:4] = coordinates[1][:]
    config.chess_coordinates[4:6] = coordinates[2][:]
    config.chess_coordinates[6:8] = coordinates[3][:]

def calibrate_white():
    global marks_updated
    global coordinates
    coordinates = []

    global img
    image.fixImg()
    img = config.transformed_img.copy()

    config.white_piece_low = [255, 255, 255]
    config.white_piece_high = [0, 0, 0]

    cv2.namedWindow('image')
    cv2.namedWindow('hsv image')
    cv2.setMouseCallback('image', mouse_callback)
    print(img.shape)
    while True:


        hsv = cv2.cvtColor(config.transformed_img.copy(), cv2.COLOR_BGR2HSV)

        mask_white = cv2.inRange(hsv, (config.white_piece_low[0], config.white_piece_low[1], config.white_piece_low[2]),
                                 (config.white_piece_high[0], config.white_piece_high[1], config.white_piece_high[2]))

        imask_white = mask_white > 0
        hsv_filter = np.zeros_like(config.transformed_img, np.uint8)
        hsv_filter[imask_white] = config.transformed_img[imask_white]

        cv2.imshow('image', img)
        cv2.imshow('hsv image', hsv_filter)

        # close everything if Esc is pressed
        k = cv2.waitKey(1)

        if k == 27:
            break

        if marks_updated:
            marks_updated = False
            config.white_piece_low[0] = int(min(hsv[coordinates[-1][1], coordinates[-1][0]][0], config.white_piece_low[0]))
            config.white_piece_low[1] = int(min(hsv[coordinates[-1][1], coordinates[-1][0]][1], config.white_piece_low[1]))
            config.white_piece_low[2] = int(min(hsv[coordinates[-1][1], coordinates[-1][0]][2], config.white_piece_low[2]))
            config.white_piece_high[0] = int(max(hsv[coordinates[-1][1], coordinates[-1][0]][0], config.white_piece_high[0]))
            config.white_piece_high[1] = int(max(hsv[coordinates[-1][1], coordinates[-1][0]][1], config.white_piece_high[1]))
            config.white_piece_high[2] = int(max(hsv[coordinates[-1][1], coordinates[-1][0]][2], config.white_piece_high[2]))
    #
    #
    # print('coordinates are: {}'.format(coordinates))
    # config.chess_coordinates = coordinates

def write_calibration():
    with open('calibration.txt', 'w') as f:
        for num in config.white_piece_low:
            f.write(str(num)+'\n')
        for num in config.white_piece_high:
            f.write(str(num)+'\n')
        for num in config.chess_coordinates:
            f.write(str(num)+'\n')
        f.close()