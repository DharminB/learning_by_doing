#! /usr/bin/env python

import os
import yaml
import cv2
import time
import random
import numpy as np

from tic_tac_toe_with_robot.board import Board
from tic_tac_toe_with_robot.homography import get_tiles

class BoardPerceiver(object):

    """Perceive tic tac toe board using opencv"""

    def __init__(self, perception_config=None, visualise=True, debug=True):
        if perception_config is None:
            print("Empty perception config received")
            sys.exit(1)
        self.perception_config = perception_config
        self.debug = debug
        self.visualise = visualise
        if self.debug:
            print(self.perception_config)
        self.video_feed = cv2.VideoCapture(1)

    def __del__(self):
        self.video_feed.release()
        # cv2.destroyAllWindows()

    def perceive_board(self):
        """Main function to perceive board
        :returns: Board obj or None

        """
        for i in range(4):
            self.video_feed.grab()
        ret, frame = self.video_feed.read()
        gray_full = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.debug:
            cv2.imshow('gray_full', gray_full)

        # tiles = self._get_all_tiles(gray_full)
        ideal_ids = np.array([11, 3, 7, 10])
        tiles = get_tiles(gray_full, ideal_ids=ideal_ids)
        if tiles is None:
            return None
        board_list = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
        for i in range(3):
            for j in range(3):
                board_list[i][j] = self._determine_tile_type(tiles[i][j], str(i)+str(j))
        board = Board.from_list(board_list)
        valid = board.is_valid()

        if self.visualise:
            self._visualise_board_on_img(board, tiles)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()

        if not valid:
            return None
        return board

    def perceive_trigger_tile(self):
        """Perceive trigger tile to check if trigger is raised. 
        Note: Trigger is basically a human placing a cross tile in the trigger spot

        :returns: bool

        """
        for i in range(4):
            self.video_feed.grab()
        ret, frame = self.video_feed.read()
        gray_full = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.debug:
            cv2.imshow('gray_full', gray_full)
            cv2.waitKey(1)

        roi = self.perception_config['trigger_roi']
        trigger_tile = gray_full[roi[0][0]:roi[1][0], roi[0][1]:roi[1][1]]
        # cv2.imshow('trigger', trigger_tile)
        tile_type = self._determine_tile_type(trigger_tile, 'trigger')
        if tile_type == 1:
            return True

        if self.visualise:
            self._visualise_trigger_on_img(gray_full, tile_type, trigger_tile)

        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        return False

    def _determine_tile_type(self, img, name=''):
        """Determine if `img` represents a cross, a circle or an empty tile
           Returns 0 for empty, 1 for cross and 2 for circle

        :img_name: cv2.Image
        :return: int

        """
        gaus_mask = self.get_adaptive_threshold(img)
        if self.debug:
            cv2.imshow('frame'+name, gaus_mask)
        if np.count_nonzero(gaus_mask == 0) < self.perception_config['min_blk_px_cnt']:
            return 0
        return 2 if self._is_circle(gaus_mask) else 1

    def _is_circle(self, img):
        bw_img = cv2.bitwise_not(img)
        _, contours, hierarchy = cv2.findContours(bw_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours[:]:
            if cnt.shape[0] < 10:
                continue
            ellipse = cv2.fitEllipse(cnt)
            if abs(ellipse[1][0] - ellipse[1][1]) < 4 and 30 < ellipse[1][0] < 50:
                rms_error = 0
                radius = (ellipse[1][0] + ellipse[1][0])/4
                for pt in cnt:
                    rms_error += abs(np.linalg.norm(pt - ellipse[0]) - radius)
                avg_rms_error = rms_error/cnt.shape[0]
                if avg_rms_error < 4:
                    return True
        return False

    def get_adaptive_threshold(self, img):
        return cv2.adaptiveThreshold(img,
                                     255,
                                     cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                     cv2.THRESH_BINARY,
                                     self.perception_config['adaptive_threshold_block_size'],
                                     self.perception_config['adaptive_threshold_C'])

    def _visualise_board_on_img(self, board, tiles):
        """Overlap board visualisation on img

        :board: Board
        :tiles: list of list of cv2.Image (size 3x3)
        :returns: None

        """
        tile_size = tiles[0][0].shape[0]
        image_size = int(3.2*tile_size)
        visualised_img = np.zeros((image_size, image_size), np.uint8)
        for i in range(3):
            for j in range(3):
                start_pt_x = int(1.1 * i * tile_size)
                end_pt_x = int(((1.1*i) + 1) * tile_size)
                start_pt_y = int(1.1 * j * tile_size)
                end_pt_y = int(((1.1*j) + 1) * tile_size)
                visualised_img[start_pt_x:end_pt_x, start_pt_y:end_pt_y] = self.get_adaptive_threshold(tiles[i][j])
                if board.board[i][j] == 1:
                    cv2.line(visualised_img, (start_pt_x, start_pt_y), (end_pt_x, end_pt_y), 127, 5)
                    cv2.line(visualised_img, (start_pt_x, end_pt_y), (end_pt_x, start_pt_y), 127, 5)
                
                if board.board[i][j] == 2:
                    cv2.circle(visualised_img,
                               (start_pt_y+int(tile_size/2), start_pt_x+int(tile_size/2)),
                               int(tile_size/2), 127, 4)
                    # cv2.circle(visualised_img, (start_pt_x+int(tile_size/2), start_pt_y+int(tile_size/2)), (end_pt_x-start_pt_x)/2, 127, 2)
        # cv2.imshow(str(i)+str(j), tiles[i][j])
        cv2.imshow('visualised', visualised_img)
        cv2.waitKey(1)

    def _visualise_trigger_on_img(self, img, tile_type, trigger_tile):
        visualised_img = img.copy()
        roi = self.perception_config['trigger_roi']
        visualised_img[roi[0][0]:roi[1][0], roi[0][1]:roi[1][1]] = self.get_adaptive_threshold(trigger_tile)
        if tile_type == 1:
            cv2.line(visualised_img, tuple(roi[0][::-1]), tuple(roi[1][::-1]), 127, 2)
        
        if tile_type == 0:
            cv2.rectangle(visualised_img, tuple(roi[0][::-1]), tuple(roi[1][::-1]), 127, 2)
        
        if tile_type == 2:
            # print((roi[0][0]+roi[1][0])/2,(roi[0][1]+roi[1][1])/2)
            cv2.circle(visualised_img, ((roi[0][1]+roi[1][1])/2,(roi[0][0]+roi[1][0])/2), (roi[1][1]-roi[0][1])/2, 127, 2)
        cv2.imshow('visualised', visualised_img)
        cv2.waitKey(1)

def get_perception_config(file_name):
    current_dir = os.path.dirname(os.path.abspath(__file__))
    main_dir = os.path.dirname(os.path.dirname(current_dir))
    config_file = os.path.join(main_dir, 'ros/config/' + file_name)
    perception_config = None
    with open(config_file, 'r') as file_obj:
        perception_config = yaml.safe_load(file_obj)
    return perception_config['perception_config']

if __name__ == "__main__":
    PERCEPTION_CONFIG = get_perception_config('perception_config.yaml')
    PERCEIVER = BoardPerceiver(PERCEPTION_CONFIG)
    # board = PERCEIVER.perceive_board()
    # print(board)
    triggered = PERCEIVER.perceive_trigger_tile()
    print(triggered)
