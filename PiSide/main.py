import chess
import cv2

import config
import image
import time
import playchess, calibrate
import numpy as np

board = chess.Board()

config.current_board = config.starting_board

img1 = cv2.imread('Images/scanlight.jpg')
pieces = []
config.initial_img = img1

def userMove():
    print(config.move_start)
    print(config.move_end)
    start = chr(ord('`') + int(config.move_start[0])+1) + str(int(config.move_start[1])+1)
    end = chr(ord('`') + int(config.move_end[0])+1) + str(int(config.move_end[1])+1)
    if playchess.make_move(start + end) == -1:
        wrongMove()


def wrongMove():
    print("found a wrong move :(")
    # TODO


def boardReady():
    return True  # TODO


def makeMove():
    comp_move = playchess.computer_move()
    print(comp_move)
    x_init = ord(comp_move[0]) - 97
    y_init = ord(comp_move[2]) - 97
    moved = config.current_board[int(comp_move[1]) - 1][int(x_init)]
    caught = config.current_board[int(comp_move[3]) - 1][int(y_init)]
    config.current_board[int(comp_move[1]) - 1][int(x_init)] = '.'
    config.current_board[int(comp_move[3]) - 1][int(y_init)] = moved
    print('{},{},{},{},{},{},{},{}'.format(x_init, int(comp_move[1]) - 1, y_init, int(comp_move[3]) - 1, 0, 180, moved,
                                           caught))
    print(config.current_board)
    # serial.Serial('COM4', 115200)
    # serial.write('{},{},{},{},{},{}'.format(1,x_init, y_init, int(comp_move[1]), int(comp_move[3]), 0))
    time.sleep(10)


def display_images():
    square_size = 500
    initial_img = cv2.resize(config.initial_img, (square_size, square_size))
    transformed_img = cv2.resize(config.transformed_img, (square_size, square_size))
    hsv_filter = cv2.resize(config.hsv_filter, (square_size, square_size))
    threshold = cv2.cvtColor(config.threshold, cv2.COLOR_GRAY2BGR)
    threshold = cv2.resize(threshold, (square_size, square_size))
    show_contours = cv2.resize(config.show_contours, (square_size, square_size))
    final_img = cv2.resize(config.final_img, (square_size, square_size))
    handImg = cv2.cvtColor(config.handImg, cv2.COLOR_GRAY2BGR)
    handImg = cv2.resize(handImg, (square_size, square_size))
    blank_square = np.zeros([square_size * 2, square_size * 3, 3], dtype=np.uint8)

    blank_square[0:square_size, 0:square_size] = initial_img
    blank_square[square_size * 0:square_size * 1, square_size * 1:square_size * 2] = transformed_img
    blank_square[square_size * 0:square_size * 1, square_size * 2:square_size * 3] = hsv_filter
    blank_square[square_size * 1:square_size * 2, square_size * 0:square_size * 1] = threshold
    blank_square[square_size * 1:square_size * 2, square_size * 1:square_size * 2] = show_contours
    blank_square[square_size * 1:square_size * 2, square_size * 2:square_size * 3] = handImg

    cv2.imshow("blanky", blank_square)

    k = cv2.waitKey(0)
    if k == 27:  # wait for ESC key to exit
        cv2.destroyAllWindows()
    elif k == ord('s'):  # wait for 's' key to save and exit
        cv2.imwrite('messigray.png', blank_square)
        cv2.destroyAllWindows()


def playChess():
    # while (playchess.game_over() is not True):
        image.fixImg()
        if (image.boardRead() == -1):
            time.sleep(1)
            pass
        image.findPieces()
        if image.findMove() == -1:
            print("whoops! no moves found!")
            return -1
        userMove()
        makeMove()
        display_images()


# calibrate.readcalibration()
# playChess()

image.getImage()
calibrate.readcalibration()
calibrate.calibrate_transform()
calibrate.write_calibration()
calibrate.readcalibration()
calibrate.calibrate_white()
calibrate.write_calibration()