#! /usr/bin/env python

from __future__ import print_function
import rospy

from brics_actuator.msg import JointPositions, JointValue

class TicTacToe(object):

    """Docstring for TicTacToe. """

    def __init__(self):
	self.joint_angles = rospy.get_param("~joint_angles", None)
        if self.joint_angles is None:
            rospy.logerr("Joint angles not defined.")
            sys.exit(1)
        self.joint_pos_pub = rospy.Publisher('/arm_1/arm_controller/position_command', JointPositions, queue_size=1)
        rospy.sleep(1)
        self.setup_arm_for_game()

    def setup_arm_for_game(self):
        success = self.move_arm_to('init_pose')
        print(success)
        success = self.move_arm_to('init_pose_1')
        print(success)
        success = self.move_arm_to('init_pose_2')
        print(success)
        success = self.move_arm_to('init_pose_3')
        print(success)
        success = self.move_arm_to('init_pose_4')
        print(success)
        success = self.move_arm_to('init_pose_5')
        print(success)

    def move_arm_to(self, pose_name):
        if pose_name not in self.joint_angles:
            return False
        joint_values = self.joint_angles[pose_name]
        print(pose_name, joint_values)
        joint_pos_msg = JointPositions()
        joint_value_msg_list = self.create_joint_value_msg_list(joint_values, 'rad')
        joint_pos_msg.positions = joint_value_msg_list
        self.joint_pos_pub.publish(joint_pos_msg)
        rospy.sleep(3)
        return True

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
    # rospy.spin()
