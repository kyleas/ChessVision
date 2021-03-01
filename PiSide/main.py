import chess
import cv2

import config
import image
import playchess

# sys.path.insert(0, '../neural-chessboard-draft')
# import main

board = chess.Board()

config.current_board = config.starting_board

img1 = cv2.imread('../neural-chessboard-draft/ChessBot/primary3.jpg')
img2 = cv2.imread(r'C:\Users\kyle\Desktop\neural-chessboard-draft\neural-chessboard-draft\ChessBot\rvision.jpg')
img3 = cv2.imread('../neural-chessboard-draft/ChessBot/move1_mod.jpg')
# img = main.detect(input=img2, output='../neural-chessboard-draft/ChessBot/plzwork.jpg')
img = img2
pieces = []

def chessMove():
    print(config.move_start)
    print(config.move_end)
    start = chr(ord('`')+config.move_start[0]) + str(config.move_start[1])
    end = chr(ord('`')+config.move_end[0]) + str(config.move_end[1])
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
    moved = config.current_board[int(comp_move[1])][int(x_init)]
    caught = config.current_board[int(comp_move[3])][int(y_init)]
    config.current_board[int(comp_move[1])][int(x_init)] = '.'
    config.current_board[int(comp_move[3])][int(y_init)] = moved
    print('{},{},{},{},{},{},{},{}'.format(x_init,comp_move[1], y_init, comp_move[3], 0, 0, moved, caught))
    print(config.current_board)
    # serial.Serial('COM4', 115200)
    # serial.write('{},{},{},{},{},{}'.format("one", "two", "three", "four", "five", "six"))

def playChess():
    #while playchess.game_over() is not True and boardReady():
        image.fixImg()
        image.findPieces(img)
        if image.findMove() == -1:
            return -1
        chessMove()
        makeMove()

playChess()