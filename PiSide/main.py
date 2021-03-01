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
    print('{},{},{},{},{},{},{},{}'.format(x_init, comp_move[1], y_init, comp_move[3], 0, 180, moved, caught))
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


#############################################################################
# FROM NEURAL CHESS BOT
import gc, os, sys, glob, argparse, utils

print("<<< \x1b[5;32;40m neural-chessboard \x1b[0m >>>")

from config import *
from utils import ImageObject
from slid import pSLID, SLID, slid_tendency  # == step 1
from laps import LAPS  # == step 2
from llr import LLR, llr_pad  # == step 3

from keras import backend as K
import cv2;

load = cv2.imread
save = cv2.imwrite


# NC_SCORE = -1

################################################################################

def layer():
    global NC_LAYER, NC_IMAGE  # , NC_SCORE

    print(utils.ribb("==", sep="="))
    print(utils.ribb("[%d] LAYER " % NC_LAYER, sep="="))
    print(utils.ribb("==", sep="="), "\n")

    # --- 1 step --- find all possible lines (that makes sense) ----------------
    print(utils.ribb(utils.head("SLID"), utils.clock(), "--- 1 step "))
    segments = pSLID(NC_IMAGE['main'])
    raw_lines = SLID(NC_IMAGE['main'], segments)
    lines = slid_tendency(raw_lines)

    # --- 2 step --- find interesting intersections (potentially a mesh grid) --
    print(utils.ribb(utils.head("LAPS"), utils.clock(), "--- 2 step "))
    points = LAPS(NC_IMAGE['main'], lines)
    # print(abs(49 - len(points)), NC_SCORE)
    # if NC_SCORE != -1 and abs(49 - len(points)) > NC_SCORE * 4: return
    # NC_SCORE = abs(49 - len(points))

    # --- 3 step --- last layer reproduction (for chessboard corners) ----------
    print(utils.ribb(utils.head(" LLR"), utils.clock(), "--- 3 step "))
    inner_points = LLR(NC_IMAGE['main'], points, lines)
    four_points = llr_pad(inner_points, NC_IMAGE['main'])  # padcrop

    # --- 4 step --- preparation for next layer (deep analysis) ----------------
    print(utils.ribb(utils.head("   *"), utils.clock(), "--- 4 step "))
    print(four_points)
    try:
        NC_IMAGE.crop(four_points)
    except:
        utils.warn("niestety, ale kolejna warstwa nie jest potrzebna")
        NC_IMAGE.crop(inner_points)

    print("\n")


################################################################################

def detect(args):
    global NC_LAYER, NC_IMAGE, NC_CONFIG

    if (not os.path.isfile(args.input)):
        utils.errn("error: the file \"%s\" does not exits" % args.input)

    NC_IMAGE, NC_LAYER = ImageObject(load(args.input)), 0
    for _ in range(NC_CONFIG['layers']):
        NC_LAYER += 1;
        layer()
    save(args.output, NC_IMAGE['orig'])

    print("DETECT: %s" % args.input)


def dataset(args):
    print("DATASET: use dataset.py")  # FIXME


def train(args):
    print("TRAIN: use train.py")  # FIXME


def test(args):
    files = glob.glob('test/in/*.jpg')

    for iname in files:
        oname = iname.replace('in', 'out')
        args.input = iname;
        args.output = oname
        detect(args)

    print("TEST: %d images" % len(files))


################################################################################

if __name__ == "__main__":
    utils.reset()

    p = argparse.ArgumentParser(description= \
                                    'Find, crop and create FEN from image.')

    p.add_argument('mode', nargs=1, type=str, \
                   help='detect | dataset | train')
    p.add_argument('--input', type=str, \
                   help='input image (default: input.jpg)')
    p.add_argument('--output', type=str, \
                   help='output path (default: output.jpg)')

    # os.system("rm test/steps/*.jpg") # FIXME: to jest bardzo grozne
    os.system("rm -rf test/steps; mkdir test/steps")

    args = p.parse_args();
    mode = str(args.mode[0])
    modes = {'detect': detect, 'dataset': dataset, 'train': train, 'test': test}

    if mode not in modes.keys():
        utils.errn("hey, nie mamy takiej procedury!!! (wybrano: %s)" % mode)

    modes[mode](args);
    print(utils.clock(), "done")
    K.clear_session();
    gc.collect()  # FIX: tensorflow#3388