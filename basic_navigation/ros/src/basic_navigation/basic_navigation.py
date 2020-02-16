from __future__ import print_function

import copy
import math
import rospy
from std_msgs.msg import Empty
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PointStamped
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped

from navfn.srv import MakeNavPlan, MakeNavPlanRequest, MakeNavPlanResponse

from utils import Utils
from global_planner_utils import GlobalPlannerUtils

class BasicNavigation(object):

    """Navigation to move in a straight line towards a goal"""

    def __init__(self):
        # Class variables
        self.goal = None
        self.curr_pos = None
        self.plan = None
        self.reached_goal_once = False
        self.moving_backward = False
        self.global_frame = rospy.get_param('~global_frame', 'map')
        self.robot_frame = rospy.get_param('~robot_frame', 'base_link')
        self.use_global_planner = rospy.get_param('~use_global_planner', False)
        self.allow_backward_motion = rospy.get_param('~allow_backward_motion', False)
        self.num_of_retries = rospy.get_param('~num_of_retries', 3)
        self.retry_attempts = 0
        self.last_applied_vel_x = 0.0

        # tolerances
        self.heading_tolerance = rospy.get_param('~heading_tolerance', 0.5)
        self.goal_dist_tolerance = rospy.get_param('~goal_dist_tolerance', 0.1)
        self.goal_theta_tolerance = rospy.get_param('~goal_theta_tolerance', 0.1)
        self.latch_xy_goal = rospy.get_param('~latch_xy_goal', True)
        self.dist_between_wp = rospy.get_param('~dist_between_wp', 1.0)
        self.waypoint_dist_tolerance = rospy.get_param('~waypoint_dist_tolerance', 0.3)
        max_safe_costmap_val = rospy.get_param('~max_safe_costmap_val', 80)

        # controller params
        self.p_theta_in_place = rospy.get_param('p_theta_in_place', 5.0)
        self.p_theta = rospy.get_param('p_theta', 1.0)
        self.p_linear = rospy.get_param('p_linear', 1.0)
        self.max_theta_vel = rospy.get_param('~max_theta_vel', 0.5)
        self.min_theta_vel = rospy.get_param('~min_theta_vel', 0.005)
        self.max_linear_vel = rospy.get_param('~max_linear_vel', 0.3)
        self.min_linear_vel = rospy.get_param('~min_linear_vel', 0.1)
        max_linear_acc_per_second = rospy.get_param('~max_linear_acc', 0.2)
        control_rate = rospy.get_param('~control_rate', 5.0)
        self.max_linear_acc = max_linear_acc_per_second/control_rate
        self.future_pos_lookahead_dist = rospy.get_param('~future_pos_lookahead_dist', 0.4)
        self.future_pos_lookahead_time = rospy.get_param('~future_pos_lookahead_time', 3.0)

        # recovery
        self.recovery_enabled = rospy.get_param('~recovery_enabled', False)
        if self.recovery_enabled:
            self.recovery_wait_duration = rospy.get_param('~recovery_wait_duration', 1.0)
            self.recovery_motion_duration = rospy.get_param('~recovery_motion_duration', 2.0)
            self.recovery_vel = rospy.get_param('~recovery_vel', -0.1)

        self.costmap_to_vel_multiplier = (self.max_linear_vel-self.min_linear_vel)/max_safe_costmap_val

        # Publishers
        self._cmd_vel_pub = rospy.Publisher('~cmd_vel', Twist, queue_size=1)
        self._path_pub = rospy.Publisher('~path', Path, queue_size=1)
        self._traj_pub = rospy.Publisher('~trajectory', Path, queue_size=1)
        self._collision_lookahead_point_pub = rospy.Publisher('~collision_point',
                                                              PointStamped,
                                                              queue_size=1)

        # Subscribers
        costmap_sub = rospy.Subscriber('~costmap', OccupancyGrid, self.costmap_cb)
        goal_sub = rospy.Subscriber('~goal', PoseStamped, self.goal_cb)
        localisation_sub = rospy.Subscriber('~localisation', PoseWithCovarianceStamped, self.localisation_cb)
        cancel_goal_sub = rospy.Subscriber('~cancel', Empty, self.cancel_current_goal)

        # Service client
        # Global planner
        if self.use_global_planner:
            self.global_planner_utils = GlobalPlannerUtils()

        rospy.sleep(0.5)
        rospy.loginfo('Initialised')

    def run_once(self):
        """
        Main event loop
        """
        if self.goal is None:
            return

        if self.curr_pos is None:
            rospy.logwarn('Current pose is not available')
            return

        if self.plan is None:
            if self.use_global_planner:
                rospy.loginfo('Trying to get global plan')
                self._get_global_plan()
                return
            else:
                rospy.loginfo('Getting straight line path')
                self._get_straight_line_plan()
                return

        curr_goal = Utils.get_x_y_theta_from_pose(self.plan[0].pose)
        dist = Utils.get_distance_between_points(self.curr_pos[:2], curr_goal[:2])
        if len(self.plan) == 1 and (dist < self.goal_dist_tolerance or (self.latch_xy_goal and self.reached_goal_once)) :
            self.reached_goal_once = True
            angular_dist = Utils.get_shortest_angle(curr_goal[2], self.curr_pos[2])
            if abs(angular_dist) < self.goal_theta_tolerance:
                rospy.loginfo('REACHED GOAL')
                self._reset_state()
                return
            else:
                self._rotate_in_place(theta_error=angular_dist)
                return
        if dist < self.waypoint_dist_tolerance and len(self.plan) > 1:
            rospy.loginfo('Reached waypoint')
            self.plan.pop(0)

        heading = math.atan2(curr_goal[1]-self.curr_pos[1], curr_goal[0]-self.curr_pos[0])
        heading_diff = Utils.get_shortest_angle(heading, self.curr_pos[2])
        if self.allow_backward_motion:
            heading_diff_backward = Utils.get_shortest_angle(heading,
                                                             Utils.get_reverse_angle(self.curr_pos[2]))
            if abs(heading_diff) > abs(heading_diff_backward):
                self.moving_backward = True
                heading_diff = heading_diff_backward
            else:
                self.moving_backward = False
        if abs(heading_diff) > self.heading_tolerance:
            self._rotate_in_place(theta_error=heading_diff)
        else:
            self._move_forward(pos_error=dist, theta_error=heading_diff)

    def _rotate_in_place(self, theta_error=1.0):
        self.last_applied_vel_x = 0.0
        theta_vel_raw = theta_error * self.p_theta_in_place
        theta_vel = Utils.clip(theta_vel_raw, self.max_theta_vel, self.min_theta_vel)
        self._cmd_vel_pub.publish(Utils.get_twist(x=0.0, y=0.0, theta=theta_vel))
        self._traj_pub.publish(self._get_traj(self.min_linear_vel, theta_vel))

    def _move_forward(self, pos_error=1.0, theta_error=1.0):
        future_vel_costmap = self._get_vel_based_on_costmap()
        if future_vel_costmap < self.min_linear_vel:
            rospy.logerr('Obstacle ahead. Current plan failed.')
            if self.retry_attempts < self.num_of_retries:
                rospy.loginfo('Retrying')
                self.retry_attempts += 1
                self.publish_zero_vel()
                if self.recovery_enabled:
                    self.recover()
                self.plan = None
            else:
                rospy.logerr('ABORTING')
                self._reset_state()
            return

        future_vel_prop_raw = pos_error * self.p_linear
        future_vel_prop = Utils.clip(future_vel_prop_raw, self.max_linear_vel, self.min_linear_vel)

        desired_x_vel = min(future_vel_costmap, future_vel_prop)

        # ramp up the vel acc. to max_linear_acc
        if desired_x_vel > self.last_applied_vel_x + self.max_linear_acc:
            x_vel = self.last_applied_vel_x + self.max_linear_acc
        else:
            x_vel = desired_x_vel
        self.last_applied_vel_x = x_vel

        if self.moving_backward:
            x_vel *= -1
        theta_vel_raw = theta_error * self.p_theta
        theta_vel = Utils.clip(theta_vel_raw, self.max_theta_vel, self.min_theta_vel)
        self._cmd_vel_pub.publish(Utils.get_twist(x=x_vel, y=0.0, theta=theta_vel))
        self._traj_pub.publish(self._get_traj(x_vel, theta_vel))

    def recover(self):
        rospy.loginfo('Recovering')
        if self.recovery_wait_duration > 0:
            rospy.loginfo('Recovery bahaviour: WAITING (duration: ' +
                          str(self.recovery_wait_duration) + ' seconds)')
            rospy.sleep(self.recovery_wait_duration)
        if self.recovery_motion_duration > 0:
            rospy.loginfo('Recovery bahaviour: MOVE BACKWARDS (duration: ' +
                          str(self.recovery_motion_duration) + ' seconds)')
            start_time = rospy.get_time()
            recovery_twist = Utils.get_twist(x=self.recovery_vel)
            if self.moving_backward:
                recovery_twist.linear.x *= -1
            while rospy.get_time() < start_time + self.recovery_motion_duration:
                rospy.sleep(0.1)
                self._cmd_vel_pub.publish(recovery_twist)

        rospy.loginfo('Recovery finished')

    def goal_cb(self, msg):
        self.goal = Utils.get_x_y_theta_from_pose(msg.pose)
        rospy.loginfo('Received new goal')
        rospy.loginfo(self.goal)
        self.moving_backward = False
        if self.plan is not None:
            self.plan = None
            rospy.logwarn('Preempting current goal. User requested another goal')

    def localisation_cb(self, msg):
        self.curr_pos = Utils.get_x_y_theta_from_pose(msg.pose.pose)

    def costmap_cb(self, msg):
        self.costmap_msg = msg

    def _get_global_plan(self):
        """Call global planner to get a plan based on current position and goal

        :returns: None

        """
        self.plan = self.global_planner_utils.get_global_plan(self.curr_pos, self.goal)
        if self.plan is None:
            rospy.logerr('Global planner failed.')
            rospy.logerr('ABORTING')
            self._reset_state()
            return

        # publish path
        path_msg = Path()
        path_msg.header.frame_id = self.global_frame
        path_msg.header.stamp = rospy.Time.now()
        path_msg.poses = self.plan
        self._path_pub.publish(path_msg)

    def _get_straight_line_plan(self):
        """
        Generate a straight line path to reach goal
        """
        self.plan = []
        path_msg = Path()
        path_msg.header.frame_id = self.global_frame
        path_msg.header.stamp = rospy.Time.now()

        start_pose = Utils.get_pose_stamped_from_frame_x_y_theta(self.global_frame,
                                                                 *self.curr_pos)
        goal_pose = Utils.get_pose_stamped_from_frame_x_y_theta(self.global_frame,
                                                                *self.goal)
        self.plan.append(start_pose)
        self.plan.append(goal_pose)
        path_msg.poses = self.plan

        self._path_pub.publish(path_msg)

    def _get_costmap_value_at(self, x=0.0, y=0.0):
        """
        # Assumption: costmap's global_frame is map
        :x: float
        :y: float

        :returns: int (between -1 and 100)
        """
        costmap_origin_x = self.costmap_msg.info.origin.position.x
        costmap_origin_y = self.costmap_msg.info.origin.position.y
        diff_x = x - costmap_origin_x
        diff_y = y - costmap_origin_y
        if diff_x < 0 or diff_y < 0:
            return -1
        i = int(round(diff_x/self.costmap_msg.info.resolution))
        j = int(round(diff_y/self.costmap_msg.info.resolution))
        if i > self.costmap_msg.info.width or j > self.costmap_msg.info.height:
            return -1
        return self.costmap_msg.data[j * self.costmap_msg.info.width + i]

    def _get_vel_based_on_costmap(self):
        current_heading = self.curr_pos[2]
        if self.moving_backward:
            current_heading = Utils.get_reverse_angle(current_heading)
        future_pos = (self.curr_pos[0] + self.future_pos_lookahead_dist * math.cos(current_heading),
                      self.curr_pos[1] + self.future_pos_lookahead_dist * math.sin(current_heading))
        collision_point = PointStamped()
        collision_point.header.stamp = rospy.Time.now()
        collision_point.header.frame_id = self.global_frame
        collision_point.point.x = future_pos[0]
        collision_point.point.y = future_pos[1]
        self._collision_lookahead_point_pub.publish(collision_point)
        future_costmap_value = self._get_costmap_value_at(*future_pos)
        future_vel = self.max_linear_vel - (future_costmap_value * self.costmap_to_vel_multiplier)
        return future_vel

    def _get_traj(self, vel_x, vel_theta):
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = self.robot_frame

        dist = abs(vel_x) * self.future_pos_lookahead_time
        angular_dist = abs(vel_theta) * self.future_pos_lookahead_time
        radius = dist/angular_dist

        sign_x = 1 if vel_x > 0 else -1
        sign_theta = 1 if vel_theta > 0 else -1

        num_of_points = 10
        theta_inc = angular_dist/num_of_points
        for i in range(num_of_points):
            theta = i * theta_inc
            pose = PoseStamped()
            pose.pose.position.x = sign_x * (radius * math.sin(theta))
            pose.pose.position.y = sign_theta * radius * (1 - math.cos(theta))
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        return path_msg

    def _reset_state(self):
        self.publish_zero_vel()
        self.goal = None
        self.plan = None
        self.reached_goal_once = False
        self.moving_backward = False
        self.retry_attempts = 0
        self.last_applied_vel_x = 0.0

    def cancel_current_goal(self, msg):
        rospy.logwarn('PREEMPTING (cancelled goal)')
        self._reset_state()

    def publish_zero_vel(self):
        self.last_applied_vel_x = 0.0
        self._cmd_vel_pub.publish(Utils.get_twist())

