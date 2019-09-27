#! /usr/bin/env python

from __future__ import print_function
import copy
import time

class AIPlayer(object):

    def __init__(self, player_num) :
        self.player_num = player_num
        self.max_depth = 5

    @staticmethod
    def get_best_move(board, player_num) :
        """ Decides which cell to put it's piece according to minimax algorithm
        or minimax with alpha beta pruning (depends on self.alpha_beta)
        Also returns the column number (for server client application, not used here)

        :board: Board obj
        :player_num: int (1 or 2)
        :returns: int, int
        """
        if board.is_empty():
            row, col = 1, 1
        else:
            ai_player = AIPlayer(player_num)
            row, col = ai_player.minmax(board, ai_player.player_num, True, 0)

        return row, col

    def minmax(self, board, player_num, index_needed, depth) :
        """ Recursive function which implements minimax algorithm if index_needed
        is given True, then returns the number of column that that is best,
        otherwise returns the value itself

        :state: Board object
        :color: character
        :index_needed: Boolean
        :depth: int
        :returns: int or (int, int)

        """

        # the program should never reach here but just to be extra safe
        if depth > self.max_depth:
            return 0

        # return if terminal state
        if board.is_game_complete():
            if board.get_winner() == self.player_num:
                return 1000*(self.max_depth - depth)
            elif board.get_winner() == 0:
                return 0
            else :
                return -1000*(self.max_depth - depth)

        # initialise values
        children_values = []
        children_index = []

        # determine next players color
        next_player_num = 3 - player_num

        # iterate through all possible actions
        for i in range(3) :
            for j in range(3):
                # if cell is occupied, don't call recursive function
                if board.board[i][j] != 0:
                    continue

                child = copy.deepcopy(board)
                child.board[i][j] = player_num

                # call recursive function with newly created board state and increased depth
                answer_from_child = self.minmax(child, next_player_num, False, depth+1)

                # save the value from child
                children_values.append(answer_from_child)
                children_index.append((i, j))

        # determine if its max's move or min's move and initialise "desired_value" accordingly
        if player_num == self.player_num :
            desired_value = max(children_values)
        else :
            desired_value =  min(children_values)

        # determine if the index of the action is needed or not and return value accordingly
        if index_needed :
            return children_index[children_values.index(desired_value)]
        else :
            return desired_value
