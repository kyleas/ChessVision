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

NC_LAYER = 0
NC_IMAGE = object
NC_CLOCK = time()
NC_DEBUG = False # True

NC_CONFIG = {'layers': 3}