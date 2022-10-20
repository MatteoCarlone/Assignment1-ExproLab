#! /usr/bin/env python

import rospy

from actionlib import SimpleActionServer

from exprolab_1.msg import Point, ControlFeedback, ControlResult

class ControllingAction(object):

    def __init__(self):

    	self._as = SimpleActionServer('motion/controller',
                                      arch_skeleton.msg.ControlAction,
                                      execute_cb=self.execute_callback,
                                      auto_start=False)


    	self._as.start()

    def execute_callback(self, goal):
        # Check if the provided plan is processable. If not, this service will be aborted.
        if goal is None or goal.via_points is None or len(goal.via_points) == 0:
            print('No via points provided! This service will be aborted!')
            self._as.set_aborted()
            return

        feedback = ControlFeedback()

        for point in goal.via_points:
            # Check that the client did not cancel this service.
            if self._as.is_preempt_requested():
                print('Service has been cancelled by the client!')
                # Actually cancel this service.
                self._as.set_preempted()
                return

        	feedback.reached_point = point
            self._as.publish_feedback(feedback)
            # Set the new current position into the `robot-state` node.

        result = ControlResult()
        result.reached_point = feedback.reached_point
        print('Motion control successes.')
        self._as.set_succeeded(result)
        return  # Succeeded.

if __name__ == '__main__':
    # Initialise the node, its action server, and wait.   
    rospy.init_node('controller', log_level=rospy.INFO)
    server = ControllingAction()
    rospy.spin()