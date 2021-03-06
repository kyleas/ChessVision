import cv2
import numpy as np
import config
import time
from picamera.array import PiRGBArray
from picamera import PiCamera

def displayImage(img):
    cv2.imshow("image", img)
    k = cv2.waitKey(0)
    if k == 27:
        cv2.destroyAllWindows()    

def fixImg():
    camera = PiCamera()
    camera.resolution = (1600,1200)
    rawCapture = PiRGBArray(camera, size=(1600,1200))

    time.sleep(0.5)

    camera.capture(rawCapture, format="bgr")
    config.initial_img = rawCapture.array
    displayImage(config.initial_img)
    cv2.imwrite('initial.jpg',config.initial_img)

#     cv2.imshow("Image", config.initial_img)
#     cv2.waitKey(0)
        
    pts1 = np.float32([config.chess_coordinates[0:2], config.chess_coordinates[2:4], config.chess_coordinates[4:6], config.chess_coordinates[6:8]])
    pts2 = np.float32([[0, 0], [1200, 0], [0, 1200], [1200, 1200]])

    M = cv2.getPerspectiveTransform(pts1, pts2)

    config.transformed_img = cv2.warpPerspective(config.initial_img, M, (1200, 1200))
    displayImage(config.transformed_img)
    cv2.imwrite('transform.jpg',config.transformed_img)

def findPieces():
    hsv = cv2.cvtColor(config.transformed_img.copy(), cv2.COLOR_BGR2HSV)

    mask_white = cv2.inRange(hsv, (config.white_piece_low[0], config.white_piece_low[1], config.white_piece_low[2]),
                             (config.white_piece_high[0], config.white_piece_high[1], config.white_piece_high[2]))

    imask_white = mask_white > 0
    config.hsv_filter = np.zeros_like(config.transformed_img, np.uint8)
    config.hsv_filter[imask_white] = config.transformed_img[imask_white]
    displayImage(config.hsv_filter)

    gray = cv2.cvtColor(config.hsv_filter, cv2.COLOR_BGR2GRAY)
    cv2.imshow('image',gray)

    ret, config.threshold = cv2.threshold(gray, 160, 255, 0)
    contours, hierarchy = cv2.findContours(config.threshold, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

    config.contours_img = np.zeros(config.threshold.shape)

    for i in range(len(contours)):
        if hierarchy[0][i][3] == -1:
            cv2.drawContours(config.contours_img, contours, i, 255, -1)

    external_contours = config.contours_img.astype('uint8')
    config.show_contours = cv2.cvtColor(external_contours, cv2.COLOR_GRAY2RGB)
    squares = []

    for i in contours:
        if cv2.contourArea(i) > 5000:
            x, y, w, h = cv2.boundingRect(i)
            cv2.rectangle(config.show_contours, (x, y), (x + w, y + h), (0, 255, 0), 2)

            cx = int(cv2.moments(i)['m10'] / cv2.moments(i)['m00'])
            cy = int(cv2.moments(i)['m01'] / cv2.moments(i)['m00'])
            cv2.circle(config.show_contours, (cx, cy), 15, (255, 0, 0), -1)
            squares.append((cx, cy))

    the_sum = 0
    max_val = 0
    max_n = 0

    for n, num in enumerate(contours):
        the_sum += cv2.contourArea(num)
        if max_val < cv2.contourArea(num):
            max_val = cv2.contourArea(num)
            max_n = n

    x = config.show_contours.shape[1]
    y = config.show_contours.shape[0]
    x_interval = int(x / 8)
    y_interval = int(x / 8)
    more_line = config.show_contours

    for i in range(8):
        cv2.line(more_line, (0 + i * x_interval, 0), (0 + i * x_interval, y), (0, 0, 255), 2)
        cv2.line(more_line, (0, 0 + i * y_interval), (x, 0 + i * y_interval), (0, 0, 255), 2)

    config.final_img = cv2.cvtColor(config.transformed_img, cv2.COLOR_BGR2RGB)
    for i in squares:
        start_x = int(i[0] / x_interval) * x_interval
        start_y = int(i[1] / y_interval) * y_interval
        cv2.circle(config.final_img, (start_x + int(x_interval / 2), start_y + int(y_interval / 2)), 15, (255, 0, 0), -1)
        config.pieces.append([int(start_x / x_interval), int(start_y / y_interval)])
    config.final_img = cv2.cvtColor(config.final_img, cv2.COLOR_RGB2BGR)

def findMove():
    captured = ''
    moved_piece = ''
    moved = False
    totalMoved = 0
    for row_index, row in enumerate(config.current_board):
        for col_index, item in enumerate(row):
            if [col_index, row_index] not in config.pieces and item.islower():
                moved_piece = item
                moved = True
                config.move_start = [col_index, row_index]
                config.current_board[row_index][col_index] = '.'
                totalMoved += 1
    print(config.pieces)
    print("{} pieces have moved!".format(totalMoved))

    if moved is not True or totalMoved > 1:
        return -1

    for spot in config.pieces:
        # print(spot)
        if (config.current_board[spot[1]][spot[0]].islower() is False) and moved:
            captured = config.current_board[spot[1]][spot[0]]
            config.current_board[spot[1]][spot[0]] = moved_piece
            config.move_end = [int(i) for i in spot]
        # print('{} is the piece at {}'.format(config.current_board[spot[1]][spot[0]],spot))
    print('The moved piece is {} and it moved from {} to {}'.format(moved_piece, config.move_start, config.move_end))
    if captured != '':
        print('And it captured {}'.format(captured))

def boardRead():
    lower = [15, 20, 70]
    upper = [25, 40, 80]

    lower = [int(lower[0] * 255 / 360), int(lower[1] * 255 / 100), int(lower[2] * 255 / 100)]
    upper = [int(upper[0] * 255 / 360), int(upper[1] * 255 / 100), int(upper[2] * 255 / 100)]

    hsvim = cv2.cvtColor(config.transformed_img, cv2.COLOR_BGR2HSV)
    lower = np.array(lower, dtype="uint8")
    upper = np.array(upper, dtype="uint8")

    skinRegionHSV = cv2.inRange(hsvim, lower, upper)
    blurred = cv2.blur(skinRegionHSV, (2, 2))
    ret, config.handImg = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY)

    newImg = config.transformed_img
    contours, hierarchy = cv2.findContours(config.handImg, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = max(contours, key=lambda x: cv2.contourArea(x))
    cv2.drawContours(newImg, [contours], -1, (255, 255, 0), 2)

    for i in contours:
        if cv2.contourArea(i) > 5000:
            return 1
    return -1