#! /usr/bin/env python

import os
import yaml
import cv2
import time
import random
import numpy as np

from tic_tac_toe_with_robot.board import Board

class BoardPerceiver(object):

    """Perceive tic tac toe board using opencv"""

    def __init__(self, perception_config=None, visualise=True, debug=False):
        if perception_config is None:
            print("Empty perception config received")
            sys.exit(1)
        self.perception_config = perception_config
        self.debug = debug
        self.visualise = visualise
        if self.debug:
            print(self.perception_config)
        self.video_feed = cv2.VideoCapture(0)

    def __del__(self):
        self.video_feed.release()

    def perceive_board(self):
        """Main function to perceive board
        :returns: TODO

        """
        ret, frame = self.video_feed.read()
        gray_full = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.debug:
            cv2.imshow('gray_full', gray_full)

        tiles = self._get_all_tiles(gray_full)
        board_list = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
        for i in range(3):
            for j in range(3):
                board_list[i][j] = self._determine_tile_type(tiles[i][j], str(i)+str(j))
        board = Board.from_list(board_list)
        print(board)

        if self.visualise:
            self._visualise_board_on_img(gray_full, board, tiles)

        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def _get_all_tiles(self, full_img):
        """Gets all the cropped tile images of tic tac toe board based on the config

        :full_img: cv2.Image
        :returns: TODO

        """
        tiles = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
        for i in range(3):
            for j in range(3):
                roi = self.perception_config['board_tiles_roi']['tile_'+str(i)+'_'+str(j)]
                tiles[i][j] = full_img[roi[0][0]:roi[1][0], roi[0][1]:roi[1][1]]
                # if self.debug:
                #     cv2.imshow('tile_'+str(i) + '_' + str(j), tiles[i][j])
        return tiles
        
    def _determine_tile_type(self, img, name=''):
        """Determine if `img` represents a cross, a circle or an empty tile
           Returns 0 for empty, 1 for cross and 2 for circle

        :img_name: cv2.Image
        :return: int

        """
        gaus_mask = BoardPerceiver.get_adaptive_threshold(img)
        if self.debug:
            cv2.imshow('frame'+name, gaus_mask)
        height, width = gaus_mask.shape
        w = self.perception_config['inner_square_width']
        gaus_crop = gaus_mask[height/2-w : height/2+w,   width/2-w : width/2+w]
        if self.debug:
            cv2.imshow('gaus_crop'+name, gaus_crop)
        # return np.any(gaus_crop == 0)
        return random.randint(0, 2)
        # return int(name[0])

    @staticmethod
    def get_adaptive_threshold(img):
        return cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                          cv2.THRESH_BINARY, 125, 1)

    def _visualise_board_on_img(self, img, board, tiles):
        """Overlap board visualisation on img

        :img: cv2.Image
        :board: Board
        :tiles: list of list of cv2.Image
        :returns: None

        """
        for i in range(3):
            for j in range(3):
                roi = self.perception_config['board_tiles_roi']['tile_'+str(i)+'_'+str(j)]
                print(roi)
                img[roi[0][0]:roi[1][0], roi[0][1]:roi[1][1]] = BoardPerceiver.get_adaptive_threshold(tiles[i][j])
                # cv2.imshow(str(i)+str(j), tiles[i][j])
                if board.board[i][j] == 1:
                    cv2.line(img, tuple(roi[0][::-1]), tuple(roi[1][::-1]), 127, 5)
                
                if board.board[i][j] == 0:
                    cv2.rectangle(img, tuple(roi[0][::-1]), tuple(roi[1][::-1]), 127, 5)
                
                if board.board[i][j] == 2:
                    print((roi[0][0]+roi[1][0])/2,(roi[0][1]+roi[1][1])/2)
                    cv2.circle(img, ((roi[0][1]+roi[1][1])/2,(roi[0][0]+roi[1][0])/2), (roi[1][1]-roi[0][1])/2, 127, 5)
        cv2.imshow('visualised', img)

def get_perception_config(file_name):
    current_dir = os.path.dirname(os.path.abspath(__file__))
    main_dir = os.path.dirname(os.path.dirname(current_dir))
    config_file = os.path.join(main_dir, 'ros/config/' + file_name)
    perception_config = None
    with open(config_file, 'r') as file_obj:
        perception_config = yaml.safe_load(file_obj)
    return perception_config['perception_config']

if __name__ == "__main__":
    # img = cv2.imread('cross.jpg', cv2.IMREAD_GRAYSCALE)
    # print(is_tile_cross(img, (250, 150, 460, 350), 30))
    # print(is_tile_cross('circle.jpg', (250, 150, 460, 350), 30))

    # cap = cv2.VideoCapture(1)
    # while True:
    #     time.sleep(0.2)
    #     ret, frame = cap.read()
    #     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #     print(is_tile_cross(gray, (200, 100, 400, 330), 30))
    #     # print(gray.shape)
    #     cv2.imshow('gray', gray)

    #     if cv2.waitKey(1) & 0xFF == ord('q'):
    #         break
    # cap.release()
    # cv2.destroyAllWindows()
    PERCEPTION_CONFIG = get_perception_config('perception_config.yaml')
    PERCEIVER = BoardPerceiver(PERCEPTION_CONFIG)
    board = PERCEIVER.perceive_board()
    print(board)
