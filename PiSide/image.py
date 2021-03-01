import cv2
import numpy as np
import config

def fixImg():
    #img = main.detect(input=img2,output='../neural-chessboard-draft/ChessBot/plzwork.jpg')
    pass

def findPieces(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h_low = 0 * 255 / 360
    h_high = 350 * 255 / 360
    s_low = 0 * 255 / 100
    s_high = 25 * 255 / 100
    v_low = 50 * 255 / 100
    v_high = 100 * 255 / 100

    mask_white = cv2.inRange(hsv, (h_low, s_low, v_low), (h_high, s_high, v_high))

    imask_white = mask_white > 0
    white = np.zeros_like(img, np.uint8)
    white[imask_white] = img[imask_white]

    gray = cv2.cvtColor(white, cv2.COLOR_BGR2GRAY)

    ret, thresh = cv2.threshold(gray, 120, 255, 0)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

    external_contours = np.zeros(thresh.shape)

    for i in range(len(contours)):
        if hierarchy[0][i][3] == -1:
            cv2.drawContours(external_contours, contours, i, 255, -1)

    external_contours = external_contours.astype('uint8')
    gray_contours = cv2.cvtColor(external_contours, cv2.COLOR_GRAY2RGB)
    squares = []

    for i in contours:
        if cv2.contourArea(i) > 5000:
            x, y, w, h = cv2.boundingRect(i)
            cv2.rectangle(gray_contours, (x, y), (x + w, y + h), (0, 255, 0), 2)

            cx = int(cv2.moments(i)['m10'] / cv2.moments(i)['m00'])
            cy = int(cv2.moments(i)['m01'] / cv2.moments(i)['m00'])
            cv2.circle(gray_contours, (cx, cy), 15, (255, 0, 0), -1)
            squares.append((cx, cy))

    the_sum = 0
    max_val = 0
    max_n = 0

    for n, num in enumerate(contours):
        the_sum += cv2.contourArea(num)
        if max_val < cv2.contourArea(num):
            max_val = cv2.contourArea(num)
            max_n = n

    x = gray_contours.shape[1]
    y = gray_contours.shape[0]
    x_interval = int(x / 8)
    y_interval = int(x / 8)
    more_line = gray_contours

    for i in range(8):
        cv2.line(more_line, (0 + i * x_interval, 0), (0 + i * x_interval, y), (0, 0, 255), 2)
        cv2.line(more_line, (0, 0 + i * y_interval), (x, 0 + i * y_interval), (0, 0, 255), 2)

    final_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    for i in squares:
        start_x = int(i[0] / x_interval) * x_interval
        start_y = int(i[1] / y_interval) * y_interval
        cv2.circle(final_img, (start_x + int(x_interval / 2), start_y + int(y_interval / 2)), 15, (255, 0, 0), -1)
        config.pieces.append([int(start_x / x_interval), int(start_y / y_interval)])
    final_img = cv2.cvtColor(final_img, cv2.COLOR_RGB2BGR)

def findMove():
    captured = ''
    moved_piece = ''
    moved = False
    for row_index, row in enumerate(config.current_board):
        for col_index, item in enumerate(row):
            if [col_index, row_index] not in config.pieces and item.islower():
                moved_piece = item
                moved = True
                config.move_start = [col_index, row_index]
    print(config.pieces)

    if moved is not True:
        return -1

    for spot in config.pieces:
        print(spot)
        if (config.current_board[spot[1]][spot[0]].islower() is False) and moved:
            captured = config.current_board[spot[1]][spot[0]]
            config.current_board[spot[1]][spot[0]] = moved_piece
            config.move_end = [int(i) for i in spot]
        print('{} is the piece at {}'.format(config.current_board[spot[1]][spot[0]],spot))
    print('The moved piece is {} and it moved from {} to {}'.format(moved_piece, config.move_start, config.move_end))
    if captured != '':
        print('And it captured {}'.format(captured))