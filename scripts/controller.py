#! /usr/bin/env python

import rospy

from actionlib import SimpleActionServer

from exprolab_1.msg import ControlAction, ControlFeedback, ControlResult

from armor_api.armor_client import ArmorClient

from exprolab_1 import environment as env

import re

import time 

class ControllingAction(object):

    def __init__(self):

        self.client = ArmorClient("armor_client", "reference")

        self._as = SimpleActionServer('motion/controller',
            ControlAction,
            execute_cb=self.execute_callback,
            auto_start=False)

        self._as.start()

    def execute_callback(self, goal):
        # Check if the provided plan is processable. If not, this service will be aborted.

        print('CONTROLLING EXECUTION')

        if goal is None or goal.point_set is None or len(goal.point_set) == 0:
            print('No via points provided! This service will be aborted!')
            self._as.set_aborted()
            return

        feedback = ControlFeedback()

        print('###############')

        for point in goal.point_set:
            # Check that the client did not cancel this service.
            if self._as.is_preempt_requested():
                print('Service has been cancelled by the client!')
                # Actually cancel this service.
                self._as.set_preempted()
                return

            print('['+ str(point.x) +' , '+ str(point.y)+']')

            feedback.reached_point = point
            self._as.publish_feedback(feedback)
            # Set the new current position into the `robot-state` node.

            rospy.sleep(0.5)

        result = ControlResult()
        result.reached_point = feedback.reached_point
        print('Motion control successes.')
        self._as.set_succeeded(result)

        starting_room = env.Map_C[str(goal.point_set[0].x) + ',' + str(goal.point_set[0].y)]
        reached_room = env.Map_C[str(result.reached_point.x) + ',' + str(result.reached_point.y)]

        print('Reached Room: '+reached_room+ ' Coordinate: '+str(result.reached_point.x) + ' , ' + str(result.reached_point.y))
    
        print('Started from Room: '+ starting_room +' Coordinate: ' + str(goal.point_set[0].x) + ' , ' + str(goal.point_set[0].y))

        self.client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', reached_room, starting_room])

        now = self.client.query.dataprop_b2_ind('now','Robot1')
        now = re.search('"(.+?)"',str(now)).group(1)

        curr_time = int(time.time())

        self.client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long' , str(curr_time)  , str(now) ])

        visited_at = self.client.query.dataprop_b2_ind('visitedAt',reached_room)

        visited_at = re.search('"(.+?)"',str(visited_at)).group(1)

        self.client.call('REPLACE','DATAPROP','IND',['visitedAt', reached_room, 'Long' , str(curr_time)  , str(visited_at) ])


        return  # Succeeded.

if __name__ == '__main__':
    # Initialise the node, its action server, and wait.   
    rospy.init_node('controller', log_level=rospy.INFO)
    server = ControllingAction()
    rospy.spin()