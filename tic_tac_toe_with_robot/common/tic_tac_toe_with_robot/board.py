#! /usr/bin/env python

from __future__ import print_function

class Board(object):

    """Board object representing board of tic tac toe
       It contains a `board` attribute which is a list of list of shape 3x3 
       containing integers. Each element can be of following type
       0 -> empty
       1 -> player 1's piece
       2 -> player 2's piece
       """

    def __init__(self):
        self.board = [[0 for i in range(3)] for j in range(3)]

    def __str__(self):
        string = ""
        for i in range(3):
            string += str(self.board[i]) + "\n"
        return string

    @staticmethod
    def from_list(board_list):
        """Initialise Board obj from a list containing int

        :board_list: list or list of list
        :returns: Board obj

        """
        if not isinstance(board_list, list) or len(board_list) == 0:
            return Board()
        #single dimentional list containing 9 elements
        if isinstance(board_list[0], int) and len(board_list) == 9:
            i, j = 0, 0
            board = Board()
            for i in range(3):
                for j in range(3):
                    board.board[i][j] = board_list[i*3 + j]
            return board
        # 2 dimentional list of shape 3x3 containing int
        elif isinstance(board_list[0], list) and isinstance(board_list[0][0], int) \
                and len(board_list) == 3 and len(board_list[0]) == 3:
            board = Board()
            board.board = [[board_list[i][j] for j in range(3)] for i in range(3)]
            return board
        else:
            return Board()

    def is_empty(self):
        """Checks if the board is empty or not
        :returns: bool

        """
        zero_count = 0
        for i in range(3):
            zero_count += self.board[i].count(0)
        return zero_count == 9

    def is_game_complete(self):
        """Checks if the game is complete or not.
        :returns: bool

        """
        completely_filled = True
        for i in range(3):
            if 0 in self.board[i]:
                completely_filled = False
                break
        if completely_filled:
            return True

        if self.get_winner() != 0:
            return True
        
        return False

    def get_winner(self):
        """Return the winner of the current board
        :returns: int (1 for player_1 and 2 for player_2 and 0 for draw or incomplete)

        """
        for i in range(3):
            if self.board[i][0] != 0 and self.board[i][0] == self.board[i][1] == self.board[i][2]:
                return self.board[i][0]
            if self.board[0][i] != 0 and self.board[0][i] == self.board[1][i] == self.board[2][i]:
                return self.board[0][i]
        if self.board[0][0] != 0 and self.board[0][0] == self.board[1][1] == self.board[2][2]:
            return self.board[0][0]
        if self.board[0][2] != 0 and self.board[0][2] == self.board[1][1] == self.board[2][0]:
            return self.board[0][2]
        return 0
