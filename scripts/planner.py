#! /usr/bin/env python

import rospy

from actionlib import SimpleActionServer

from exprolab_1.msg import Point, PlanAction, PlanFeedback, PlanResult

from armor_api.armor_client import ArmorClient

from exprolab_1 import environment as env

import numpy as np

import re

class PlaningAction(object):

    def __init__(self):

        self._environment_size = rospy.get_param(env.ENV_SIZE)

        self.client = ArmorClient('armor_client', 'reference')

		# Instantiate and start the action server based on the `SimpleActionServer` class.
        self._as = SimpleActionServer(env.ACTION_PLANNER, 
                                      PlanAction, 
                                      execute_cb=self.execute_callback, 
                                      auto_start=False)

        self._as.start()

    def execute_callback(self, goal):

        print('\n###############\nPLANNING EXECUTION')

        # starting point, could be IsIn from Reasoner Node, _get_pose_client to be implemented

        start_room = self.client.query.objectprop_b2_ind('isIn','Robot1')

        start_room = re.search('#(.+?)>',start_room[0]).group(1)

        start_point = env.Map_R[start_room]

    	# goal point
        target_point = env.Map_R[goal.target]

        log_msg = (f'Starting-Room [{start_point[0]}, {start_point[1]}] , Target-Room [{target_point[0]}, '
                       f'{target_point[1]}] ')
        print(log_msg)

        if start_point is None or target_point is None:

            log_msg = 'Cannot have `None` start point nor target_point. This service will be aborted!.'
            print(log_msg)
            # Close service by returning an `ABORT` state to the client.
            self._as.set_aborted()
            return

        # _is_valid function to be implemented
        if not(self._is_valid(start_point) and self._is_valid(target_point)):
            log_msg = (f'Start point ({start_point[0]}, {start_point[1]}) or target point ({target_point[0]}, '
                       f'{target_point[1]}) point out of the environment. This service will be aborted!.')
            print(log_msg)
            # Close service by returning an `ABORT` state to the client.
            self._as.set_aborted()
            return

        # Initialise the `feedback` with the starting point of the plan.
        feedback = PlanFeedback()
        feedback.via_points = []

        # number of via points
        n_points = 10

        points_x = np.linspace(start_point[0],target_point[0],num = n_points)
        points_y = np.linspace(start_point[1],target_point[1],num = n_points)

        points = [[a , b] for a, b in zip(points_x, points_y)]

        print('GENERATING VIA POINTS...')

        for i in range(n_points):

            if self._as.is_preempt_requested():
                print('Server has been cancelled by the client!')
                # Actually cancel this service.
                self._as.set_preempted()  
                return

            new_point = Point()
            new_point.x = points[i][0]
            new_point.y = points[i][1]

            print('[' + str("%.2f"% new_point.x)+','+str("%.2f"% new_point.y)+']')

            feedback.via_points.append(new_point)

            self._as.publish_feedback(feedback)

            rospy.sleep(0.1)

        # Publish the results to the client.        
        result = PlanResult()
        result.via_points = feedback.via_points

        self._as.set_succeeded(result)


    def _is_valid(self, point):
        
        return self._environment_size[0] <= point[0] <= self._environment_size[1] and self._environment_size[2] <= point[1] <= self._environment_size[3]


if __name__ == '__main__':

    # Initialise the node, its action server, and wait.    
    rospy.init_node(env.NODE_PLANNER, log_level=rospy.INFO)
    server = PlaningAction()
    rospy.spin()