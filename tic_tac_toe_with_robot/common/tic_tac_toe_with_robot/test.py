#! /usr/bin/env python

from __future__ import print_function
from tic_tac_toe_with_robot.board import Board
from tic_tac_toe_with_robot.player import AIPlayer

def play_game_with_comp():
    board = Board()
    print("You are PLAYER 2")
    player = 1
    while not board.is_game_complete():
        if player == 1:
            ans = AIPlayer.get_best_move(board, 1)
            board.board[ans[0]][ans[1]] = 1
        else:
            row = input("Row: ")
            col = input("Col: ")
            board.board[row][col] = 2
        player = 3 - player
        print(board)
    winner = board.get_winner()
    if winner == 0:
        print("Game draw")
    else:
        print("Winner: player ", winner)

def main():
    # board_list = [0, 1, 2, 2, 1, 0, 0, 0, 0]
    board_list = [[0, 1, 2], [2, 1, 0], [0, 0, 0]]
    board = Board.from_list(board_list)
    print(board)
    ans = AIPlayer.get_best_move(board, 1)
    print(ans)

if __name__ == "__main__":
    # main()
    play_game_with_comp()
