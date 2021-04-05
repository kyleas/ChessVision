from stockfish import Stockfish
import config
import chess
import copy

board = chess.Board()

#Windows
# stockfish = Stockfish('stockfish/stockfish_13_win_x64_bmi2.exe')
#Linux
stockfish = Stockfish('/home/pi/Downloads/Stockfish-sf_13/src/stockfish')


stockfish.set_skill_level(25)
# stockfish.set_position(["e2e4", "e7e6"])
# print(stockfish.get_board_visual())
# print(stockfish.is_move_correct('a2a5'))
# print(stockfish.get_best_move_time(250))
# stockfish.set_position(["b2b3"])
# print(stockfish.get_board_visual())
# print(stockfish.get_fen_position())


def make_move(move):
    print('the move {} is a move'.format(move))
    if stockfish.is_move_correct(move):
        config.moves.append(move)
        stockfish.set_position(config.moves)
        board = chess.Board(stockfish.get_fen_position())
        config.confirm_move = copy.deepcopy(config.current_board)
        return 1
    config.current_board = copy.deepcopy(config.confirm_move)
    config.previous_board = copy.deepcopy(config.confirm_move)
    return -1


def computer_move():
    config.moves.append((stockfish.get_best_move_time(1000)))
    stockfish.set_position(config.moves)
    board = chess.Board(stockfish.get_fen_position())
    return config.moves[-1]


def game_over():
    return board.is_stalemate() or board.is_game_over()
