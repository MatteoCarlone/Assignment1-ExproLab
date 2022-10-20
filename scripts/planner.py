#! /usr/bin/env python

import rospy

from actionlib import SimpleActionServer

from exprolab_1.msg import PlanAction, PlanFeedback, PlanResult

from armor_api.armor_client import ArmorClient

from exprolab_1 import environment as env

import numpy as np

import re

class PlaningAction(object):

    def __init__(self):

        self._environment_size = [-2,2,0,3]

        self.client = ArmorClient("armor_client", "reference")

		# Instantiate and start the action server based on the `SimpleActionServer` class.
        self._as = SimpleActionServer('motion/planner', 
                                      PlanAction, 
                                      execute_cb=self.execute_callback, 
                                      auto_start=False)

        self._as.start()

    def execute_callback(self, goal):

        # starting point, could be IsIn from Reasoner Node, _get_pose_client to be implemented

        start_point = self.client.query.objectprop_b2_ind('isIn','robot')

        print(start_point)

        start_point = re.search('#(.+?)>',start_point[0]).group(1)

        print(start_point)

        start_point = env.Map[start_point]

        print(start_point)

    	# goal point
        target_point = env.Map[goal.target]

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

        # here a should define a number of via points
        n_points = 10

        points_x = np.linspace(start_point[0],target_point[0],num = n_points)
        points_y = np.linspace(start_point[1],target_point[1],num = n_points)

        points = [a + b for a, b in zip(points_x, points_y)]

        for i in range(n_points):

            if self._as.is_preempt_requested():
                print('Server has been cancelled by the client!')
                # Actually cancel this service.
                self._as.set_preempted()  
                return

            feedback.via_points.append(points[i])

            print(feedback.via_points)

            self._as.publish_feedback(feedback)

            rospy.sleep(1)

        # Publish the results to the client.        
        result = PlanResult()
        result.via_points = feedback.via_points

        self._as.set_succeeded(result)

        log_msg = 'Motion plan succeeded'
        
        print(log_msg)


    def _is_valid(self, point):
        
        return self._environment_size[0] <= point[0] <= self._environment_size[1] and self._environment_size[2] <= point[1] <= self._environment_size[3]


if __name__ == '__main__':

    # Initialise the node, its action server, and wait.    
    rospy.init_node('planner', log_level=rospy.INFO)
    server = PlaningAction()
    rospy.spin()