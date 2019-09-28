#! /usr/bin/env python

from __future__ import print_function
import rospy

from actionlib import SimpleActionClient
from brics_actuator.msg import JointPositions, JointValue
from moveit_msgs.msg import MoveGroupAction
from mir_audio_receiver.msg import AudioMessage
import moveit_commander

from tic_tac_toe_with_robot.board import Board
from tic_tac_toe_with_robot.player import AIPlayer
from tic_tac_toe_with_robot.board_perceiver import BoardPerceiver

class TicTacToe(object):

    """Docstring for TicTacToe. """

    def __init__(self):
	self.joint_angles = rospy.get_param("~joint_angles", None)
	self.gripper_positions = rospy.get_param("~gripper_positions", None)
	perception_config = rospy.get_param("~perception_config", None)
        if self.joint_angles is None or self.gripper_positions is None or perception_config is None:
            rospy.logerr("Joint angles and/or gripper positions and/or perception config not defined.")
            sys.exit(1)

        move_group = 'move_group'
        arm = 'arm_1'

        # Wait for MoveIt!
        client = SimpleActionClient(move_group, MoveGroupAction)
        rospy.loginfo("Waiting for '{0}' server".format(move_group))
        client.wait_for_server()
        rospy.loginfo("Found server '{0}'".format(move_group))

        # Set up MoveIt!
        self.arm = moveit_commander.MoveGroupCommander(arm)

        self.joint_pos_pub = rospy.Publisher('/arm_1/arm_controller/position_command', JointPositions, queue_size=1)
        rospy.sleep(1)
        self.gripper_pos_pub = rospy.Publisher('/arm_1/gripper_controller/position_command', JointPositions, queue_size=1)
        rospy.sleep(1)
        self.audio_pub = rospy.Publisher('/mir_audio_receiver/tts_request', AudioMessage, queue_size=1)
        rospy.sleep(1)
        # self.setup_arm_for_game()
        self.move_gripper_to('open')
        rospy.sleep(1)
        # self.test_all_poses()
        # self.pick_up_tile()
        # self.place_tile(i=0, j=1)
        # self.move_gripper_to('open')
        self.perceiver = BoardPerceiver(perception_config, visualise=True, debug=False)

    def play(self):
        """Infinite loop for playing and demo
        :returns: None

        """
        """
        possible states
            wait_for_trigger
            make_a_move
            wait_for_new_game_setup
        """
        state = 'wait_for_new_game_setup'
        rospy.loginfo('Current state: wait_for_new_game_setup')
        while not rospy.is_shutdown():
            rospy.sleep(2)
            if state == 'wait_for_trigger':
                if self.perceiver.perceive_trigger_tile():
                    state = 'make_a_move'
                    rospy.loginfo('State change: wait_for_trigger -> make_a_move')
                    self.audio_pub.publish(AudioMessage(message='Wait complete'))
                    continue
            elif state == 'make_a_move':
                board = self.perceive_valid_board()
                ans = AIPlayer.get_best_move(board, 1)
                rospy.loginfo('My move: ' + str(ans))
                self.audio_pub.publish(AudioMessage(message='Making my move'))
                self.pick_up_tile()
                self.place_tile(i=ans[0], j=ans[1])
                self.move_arm_to('rest_pose')
                board = self.perceive_valid_board()
                if board.is_game_complete():
                    winner = board.get_winner()
                    rospy.loginfo('Winner: ' + str(winner))
                    if winner == 1:
                        for i in range(3):
                            self.audio_pub.publish(AudioMessage(message='Ha Ha Ha I win'))
                            rospy.sleep(2)
                    elif winner == 2:
                        for i in range(3):
                            self.audio_pub.publish(AudioMessage(message='Oh no you win'))
                            rospy.sleep(2)
                    else:
                        self.audio_pub.publish(AudioMessage(message='It was a draw. Let us play again'))
                    state = 'wait_for_new_game_setup'
                    rospy.loginfo('State change: make_a_move -> wait_for_new_game_setup')
                    continue
                state = 'wait_for_trigger'
                rospy.loginfo('State change: make_a_move -> wait_for_trigger')
                self.audio_pub.publish(AudioMessage(message='Please make your move'))
            elif state == 'wait_for_new_game_setup':
                board = self.perceive_valid_board()
                if board.is_empty():
                    rospy.loginfo('Found empty board. New game started.')
                    state = 'wait_for_trigger'
                    rospy.loginfo('State change: wait_for_new_game_setup -> wait_for_trigger')
                    self.audio_pub.publish(AudioMessage(message='Found empty board. New game started.'))
                else:
                    rospy.logwarn('Please clear the board for new game')
                    self.audio_pub.publish(AudioMessage(message='Please clear the board for new game'))

    def perceive_valid_board(self):
        board = self.perceiver.perceive_board()
        while board is None and not rospy.is_shutdown():
            rospy.logwarn('Perceived invalid board. Retrying...')
            self.audio_pub.publish(AudioMessage(message='Invalid board found'))
            board = self.perceiver.perceive_board()
            rospy.sleep(2)
        return board

    def setup_arm_for_game(self):
        self.move_arm_to('init_pose')
        self.move_arm_to('init_pose_1')
        self.move_arm_to('init_pose_2')
        self.move_arm_to('init_pose_3')
        self.move_arm_to('init_pose_4')
        self.move_arm_to('rest_pose')

    def test_all_poses(self):
        """Move arm to all the game related poses. Implemented for testing.
        :returns: None

        """
        self.move_arm_to('tile_intermediate')
        for i in range(3):
            for j in range(3):
                tile_name = 'tile_' + str(i) + '_' + str(j)
                self.move_arm_to(self.get_appropriate_intermediate(tile_name))
                self.move_arm_to(tile_name)
                self.move_arm_to(self.get_appropriate_intermediate(tile_name))
        tile_name = 'trigger_tile'
        self.move_arm_to(self.get_appropriate_intermediate(tile_name))
        self.move_arm_to('trigger_tile')
        self.move_arm_to(self.get_appropriate_intermediate(tile_name))
        self.move_arm_to('tile_middle_intermediate')
        self.move_arm_to('rest_pose')

    def pick_up_tile(self):
        """Pick a tile from trigger tile pose
        :returns: None

        """
        self.move_gripper_to('open')
        self.move_arm_to('tile_middle_intermediate')
        tile_name = 'trigger_tile'
        self.move_arm_to(self.get_appropriate_intermediate(tile_name))
        self.move_arm_to(tile_name)
        self.move_gripper_to('close')
        self.move_arm_to(self.get_appropriate_intermediate(tile_name))

    def place_tile(self, i=1, j=1):
        """Place a picked up tile to tile pose in grid
        :returns: None

        """
        tile_name = 'tile_' + str(i) + '_' + str(j)
        self.move_arm_to(self.get_appropriate_intermediate(tile_name))
        self.move_arm_to(tile_name)
        self.move_gripper_to('open')
        self.move_arm_to(self.get_appropriate_intermediate(tile_name))

    def move_arm_to(self, pose_name):
        if pose_name not in self.joint_angles:
            return False
        joint_values = self.joint_angles[pose_name]
        print(pose_name, joint_values)
        try:
            self.arm.set_joint_value_target(joint_values)
        except Exception as e:
            rospy.logerr('unable to set target position: %s' % (str(e)))
            return False
        status = self.arm.go(wait=True)
        if not status:
            rospy.logwarn("Failed moving arm with moveit, trying position command")
            joint_pos_msg = JointPositions()
            joint_value_msg_list = self.create_joint_value_msg_list(joint_values, 'rad')
            joint_pos_msg.positions = joint_value_msg_list
            self.joint_pos_pub.publish(joint_pos_msg)
            rospy.sleep(3)
            return True

    def move_gripper_to(self, pose_name):
        if pose_name not in self.gripper_positions:
            return False
        joint_values = self.gripper_positions[pose_name]
        print(pose_name, joint_values)
        joint_uris = ['gripper_finger_joint_l', 'gripper_finger_joint_r']
        joint_pos_msg = JointPositions()
        joint_value_msg_list = []
        for i in range(2):
            joint_value = JointValue(joint_uri=joint_uris[i], unit='m', value=joint_values[i])
            joint_value_msg_list.append(joint_value)
        joint_pos_msg.positions = joint_value_msg_list
        self.gripper_pos_pub.publish(joint_pos_msg)
        rospy.sleep(3)
        return True

    def get_appropriate_intermediate(self, tile_name):
        if tile_name == 'trigger_tile':
            return 'trigger_tile_intermediate'
        if len(tile_name) >= 6 and tile_name[5] == '0':
            return 'tile_left_intermediate'
        if len(tile_name) >= 6 and tile_name[5] == '1':
            return 'tile_middle_intermediate'
        if len(tile_name) >= 6 and tile_name[5] == '2':
            return 'tile_right_intermediate'
        else:
            return 'tile_middle_intermediate'

    def create_joint_value_msg_list(self, joint_values, unit):
        joint_value_msg_list = []
        for i, joint_value in enumerate(joint_values):
            joint_value_msg = JointValue()
            joint_value_msg.joint_uri = 'arm_joint_' + str(i+1)
            joint_value_msg.unit = unit
            joint_value_msg.value = joint_value
            joint_value_msg_list.append(joint_value_msg)
        return joint_value_msg_list

if __name__ == '__main__' :
    rospy.init_node('initialising_arm')
    print("inside main")
    rospy.sleep(3)
    TTT = TicTacToe()
    TTT.play()
    # rospy.spin()
