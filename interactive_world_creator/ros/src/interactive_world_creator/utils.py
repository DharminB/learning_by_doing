from __future__ import print_function

import tf
import math
import rospy
from geometry_msgs.msg import PoseStamped, Pose, TransformStamped, Quaternion
from geometry_msgs.msg import Twist

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
    def get_2_dof_interactive_marker(marker_name, frame, x=0.0, y=0.0):
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
        # int_marker.description = "Simple 2-DOF Control"

        # create a grey box marker
        box_marker = Marker()
        box_marker.type = Marker.SPHERE
        box_marker.scale.x = box_marker.scale.y = box_marker.scale.z = 0.1
        box_marker.color.r = box_marker.color.a = 1.0
        box_marker.color.g = box_marker.color.b = 0.0

        # create a non-interactive control which contains the box
        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.markers.append( box_marker )

        # add the control to the interactive marker
        int_marker.controls.append( box_control )

        # create a control which will move the box
        # this control does not contain any markers,
        # which will cause RViz to insert two arrows
        rotate_control = InteractiveMarkerControl()
        rotate_control.name = "move_x"
        rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

        # add the control to the interactive marker
        int_marker.controls.append(rotate_control);

        rotate_control2 = InteractiveMarkerControl()
        rotate_control2.orientation.z = rotate_control2.orientation.w = 0.707
        rotate_control2.name = "move_y"
        rotate_control2.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

        # add the control to the interactive marker
        int_marker.controls.append(rotate_control2);
        return int_marker
