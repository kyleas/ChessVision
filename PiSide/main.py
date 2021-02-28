# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
from typing import List

import cv2
import numpy as np
# import matplotlib.pyplot as plt
import chess
import config
import playchess
import sys
# sys.path.insert(0, '../neural-chessboard-draft')
# import main
import serial


board = chess.Board()



config.current_board = config.starting_board

img1 = cv2.imread('../neural-chessboard-draft/ChessBot/primary3.jpg')
img2 = cv2.imread(r'C:\Users\kyle\Desktop\neural-chessboard-draft\neural-chessboard-draft\ChessBot\picam.jpg')
img3 = cv2.imread('../neural-chessboard-draft/ChessBot/move1_mod.jpg')
# img = main.detect(input=img2, output='../neural-chessboard-draft/ChessBot/plzwork.jpg')
img = img2
pieces = []

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
        pieces.append([int(start_x / x_interval), int(start_y / y_interval)])
    final_img = cv2.cvtColor(final_img, cv2.COLOR_RGB2BGR)

def findMove():
    global move_end
    global move_start
    captured = ''
    moved_piece = ''
    moved = False
    for row_index, row in enumerate(config.current_board):
        for col_index, item in enumerate(row):
            if [col_index, row_index] not in pieces and item.islower():
                moved_piece = item
                moved = True
                move_start = [col_index, row_index]
    print(pieces)
    for spot in pieces:
        print(spot)
        if (config.current_board[spot[1]][spot[0]].islower() is False) and moved:
            captured = config.current_board[spot[1]][spot[0]]
            config.current_board[spot[1]][spot[0]] = moved_piece
            move_end = [int(i) for i in spot]
        print('{} is the piece at {}'.format(config.current_board[spot[1]][spot[0]],spot))
    print('The moved piece is {} and it moved from {} to {}'.format(moved_piece, move_start, move_end))
    if captured != '':
        print('And it captured {}'.format(captured))

def chessMove():
    print(move_start)
    print(move_end)
    start = chr(ord('`')+move_start[0]) + str(move_start[1])
    end = chr(ord('`')+move_end[0]) + str(move_end[1])
    if playchess.make_move(start+end) == -1:
        wrongMove()

def wrongMove():
    pass
    # TODO

def boardReady():
    return True #TODO

def makeMove():
    comp_move = playchess.computer_move()
    print(comp_move)
    x_init = ord(comp_move[0]) - 96
    y_init = ord(comp_move[2]) - 96
    print('{},{},{},{},{},{}'.format(x_init,comp_move[1], y_init, comp_move[3], "five", "six"))
    # serial.Serial('COM4', 115200)
    # serial.write('{},{},{},{},{},{}'.format("one", "two", "three", "four", "five", "six"))

def playChess():
    #while playchess.game_over() is not True and boardReady():
        fixImg()
        findPieces(img)
        findMove()
        chessMove()
        makeMove()

playChess()