from __future__ import print_function

import tf
import math
import rospy
from geometry_msgs.msg import PoseStamped, Pose, TransformStamped, Quaternion
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerControl

class Utils(object):

    """Utility functions used for interactive world creator"""

    @staticmethod
    def get_pose_from_x_y_theta(x, y, theta):
        """Return a Pose object from x, y and theta
        :x: float
        :y: float
        :theta: float
        :returns: geometry_msgs.Pose

        """
        pose = Pose()
        pose.position.x = x
        pose.position.y = y

        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, theta)
        pose.orientation = Quaternion(*quat)
        return pose

    @staticmethod
    def get_pose_stamped_from_frame_x_y_theta(frame, x, y, theta):
        """Return a Pose object from x, y and theta
        :x: float
        :y: float
        :theta: float
        :frame: string
        :returns: geometry_msgs.PoseStamped

        """
        pose = PoseStamped()
        pose.pose = Utils.get_pose_from_x_y_theta(x, y, theta)
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = frame
        return pose

    @staticmethod
    def get_x_y_theta_from_pose(pose):
        """Return a tuple(x, y, theta) from Pose objects

        :pose: geometry_msgs/Pose
        :returns: tuple(x, y, theta)

        """
        quat = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        theta = tf.transformations.euler_from_quaternion(quat)[2]
        return (pose.position.x, pose.position.y, theta)

    @staticmethod
    def get_3_dof_interactive_marker(marker_name, frame, x=0.0, y=0.0, theta=0.0):
        """Return an interactive marker with 2 degree of freedom (X and Y axis)
        in `frame` at (`x`, `y`, 0.0) position named `name`

        :marker_name: string
        :frame: string
        :x: int
        :y: int
        :returns: visualization_msgs.InteractiveMarker

        """
        # create an interactive marker for our server
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = frame
        int_marker.name = marker_name
        int_marker.pose.position.x = x
        int_marker.pose.position.y = y
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, theta)
        int_marker.pose.orientation = Quaternion(*quat)
        int_marker.description = marker_name

        # create a white box marker
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.scale.x = 1.0
        box_marker.scale.y = 0.1
        box_marker.scale.z = 1.0
        box_marker.color.r = box_marker.color.a = box_marker.color.g = box_marker.color.b = 1.0
        box_marker.pose.position.z = -0.5
        box_marker.pose.orientation.w = 1.0

        # create a non-interactive control which contains the box
        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.markers.append(box_marker)
        box_control.name = "move_x_y"
        box_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        box_control.orientation.w = box_control.orientation.y = 1.0
        int_marker.controls.append(box_control)

        rotate_control = InteractiveMarkerControl()
        rotate_control.name = "rotate_z"
        rotate_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        rotate_control.orientation.y = rotate_control.orientation.w = 1.0
        int_marker.controls.append(rotate_control);

        return int_marker
