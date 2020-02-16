from __future__ import print_function

import rospy

from navfn.srv import MakeNavPlan, MakeNavPlanRequest, MakeNavPlanResponse
from utils import Utils

class GlobalPlannerUtils(object):

    """Utils class to interact with global planner or navfn node"""

    def __init__(self, **kwargs):
        self.global_frame = rospy.get_param('~global_frame', 'map')
        self.dist_between_wp = rospy.get_param('~dist_between_wp', 1.0)
        self.heading_based_downsample = rospy.get_param('~heading_based_downsample', True)
        self.downsample_heading_tolerance = rospy.get_param('~downsample_heading_tolerance', 0.1)

        global_planner_service_topic = rospy.get_param('~global_planner_service', 'make_plan')
        rospy.loginfo('Waiting for global planner')
        rospy.wait_for_service(global_planner_service_topic)
        rospy.loginfo('Wait complete.')
        self._call_global_planner = rospy.ServiceProxy(global_planner_service_topic, MakeNavPlan)

    def get_global_plan(self, start, end):
        """Call global planner to get a plan based on given start and end positions
        Returns None on failure

        :start: list/tuple of 3 floats
        :end: list/tuple of 3 floats
        :returns: nav_msgs.Path or None

        """
        plan = []
        req = MakeNavPlanRequest()
        req.start = Utils.get_pose_stamped_from_frame_x_y_theta(self.global_frame, *start)
        req.goal = Utils.get_pose_stamped_from_frame_x_y_theta(self.global_frame, *end)
        try:
            response = self._call_global_planner(req)
            if len(response.path) > 0:
                plan = response.path
            else:
                return None
        except rospy.ServiceException as e:
            rospy.logerr(str(e))
            return None

        first_pose = Utils.get_x_y_theta_from_pose(plan[0].pose)
        second_pose = Utils.get_x_y_theta_from_pose(plan[1].pose)
        avg_dist = Utils.get_distance_between_points(first_pose[:2], second_pose[:2])
        index_offset = int(round(self.dist_between_wp/avg_dist))

        downsampled_plan = []
        for i in range(0, len(plan), index_offset):
            downsampled_plan.append(plan[i])
        downsampled_plan.append(plan[-1])

        if not self.heading_based_downsample:
            return downsampled_plan

        downsampled_plan_2 = self.downsample_based_on_heading(downsampled_plan)
        return downsampled_plan_2

    def downsample_based_on_heading(self, plan):
        downsampled_plan = []
        downsampled_plan.append(plan[0]) # first wp
        i = 0
        while i < len(plan)-1:
            print(i)
            first = (plan[i].pose.position.x, plan[i].pose.position.y)
            second = (plan[i+1].pose.position.x, plan[i+1].pose.position.y)
            initial_angle = Utils.get_line_angle(first, second)
            for j in range(i+2, len(plan)):
                new_second = (plan[j].pose.position.x, plan[j].pose.position.y)
                new_angle_formed = Utils.get_line_angle(first, new_second)
                angle_diff = abs(new_angle_formed-initial_angle)
                if angle_diff > self.downsample_heading_tolerance:
                    downsampled_plan.append(plan[j-1])
                    i = j-1
                    break
                if j == len(plan)-1:
                    downsampled_plan.append(plan[j])
                    i = j

        print('downsample', len(plan), '->', len(downsampled_plan))
        return downsampled_plan
