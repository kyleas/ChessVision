starting_board = [['r', 'n', 'b', 'q', 'k', 'b', 'n', 'r'],
                  ['p', 'p', 'p', 'p', 'p', 'p', 'p', 'p'],
                  ['.', '.', '.', '.', '.', '.', '.', '.'],
                  ['.', '.', '.', '.', '.', '.', '.', '.'],
                  ['.', '.', '.', '.', '.', '.', '.', '.'],
                  ['.', '.', '.', '.', '.', '.', '.', '.'],
                  ['P', 'P', 'P', 'P', 'P', 'P', 'P', 'P'],
                  ['R', 'N', 'B', 'K', 'Q', 'B', 'N', 'R']]

current_board = [[],[],[],[],[],[],[],[]]

moves = []

from time import time
import cv2

move_end = []
move_start = []
pieces = []

initial_img = cv2.imread('Images/initial.jpg')
transformed_img = None
hsv_filter = None
threshold = None
contours_img = None
show_contours = None
final_img = None
handImg = None

white_piece_low = []
white_piece_high = []
chess_coordinates = []