from stockfish import Stockfish
import config
import chess

board = chess.Board()

stockfish = Stockfish(r"C:\Users\kyle\Desktop\ChessVision\ChessVision\PiSide\stockfish\stockfish_13_win_x64_bmi2.exe")

stockfish.set_skill_level(25)
stockfish.set_position(["e2e4", "e7e6"])
print(stockfish.get_board_visual())
print(stockfish.is_move_correct('a2a5'))
print(stockfish.get_best_move_time(250))
stockfish.set_position(["b2b3"])
print(stockfish.get_board_visual())
print(stockfish.get_fen_position())

def make_move(move):
    if stockfish.is_move_correct(move):
        config.moves.append(move)
        stockfish.set_position(config.moves)
        board = chess.Board(stockfish.get_fen_position())
        return 1
    return -1

def computer_move():
    config.moves.append((stockfish.get_best_move_time(1000)))
    stockfish.set_position(config.moves)
    board = chess.Board(stockfish.get_fen_position())
    return config.moves[-1]

def game_over():
    return board.is_stalemate() or board.is_game_over()