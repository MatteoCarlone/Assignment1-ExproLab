#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import time
import string

from exprolab_1.msg import PlanGoal

from exprolab_1.helper import InterfaceHelper

r_to_point = ''

class Initial(smach.State):

    def __init__(self,interfacehelper):

        print('init Initial')

        smach.State.__init__(self, 
                             outcomes=['start','reasoned'],
                             input_keys=['initial_counter_in'],
                             output_keys=['initial_counter_out'])

        self._helper = interfacehelper

    def execute(self, userdata):

        print('execute')

        self._helper.start_client('load') 

        print('load request sent')
    	
    	# basic structure

        while not rospy.is_shutdown():

            # acquire mutex

            self._helper.mutex.acquire()

            try:

    			# look for transition flags
                print('checking flag')

                if self._helper.should_reasoning_start():

                    self._helper.reset_states()

                    print('reset flags and transition')

                    return 'start'

            finally:

    			# release mutex

                self._helper.mutex.release()

            rospy.sleep(0.3)


class Reasoning(smach.State):

    def __init__(self,interfacehelper):
        
        smach.State.__init__(self, 
                             outcomes=['start','reasoned'],
                             input_keys=['reasoning_counter_in'],
                             output_keys=['reasoning_counter_out'])

        self._helper = interfacehelper

    def execute(self, userdata):

        global r_to_point

        print('REASONING')

        self._helper.reason_client('reason')

        while not rospy.is_shutdown():

            # acquire mutex

            self._helper.mutex.acquire()

            try:

                # look for transition flags
                print('checking flag')

                if self._helper.should_pointing_start():

                    self._helper.reset_states()

                    print('reset flags and transition')

                    r_to_point = self._helper.room_togo()

                    print(self._helper.room_togo())

                    return 'reasoned'

            finally:

                # release mutex

                self._helper.mutex.release()

            rospy.sleep(0.3)


class Pointing(smach.State):

    def __init__(self,interfacehelper):
        
        smach.State.__init__(self, 
                             outcomes=['pointed','reasoned'],
                             input_keys=['moving_counter_in'],
                             output_keys=['moving_counter_out'])

        self._helper = interfacehelper

    def execute(self, userdata):

        global r_to_point

        print('#### POINTING ####')

        goal = PlanGoal(target= r_to_point)

        self._helper.planner_client.send_goal(goal)

        while not rospy.is_shutdown():

            # acquire mutex

            self._helper.mutex.acquire()

            try:

                # look for transition flags
                print('checking flag')

                if self._helper.planner_client.is_done():

                    print('### DONE ###')

                    self._helper.reset_states()

                    print('reset flags and transition')

                    return 'pointed'

            finally:

                # release mutex

                self._helper.mutex.release()

            rospy.sleep(0.3)

'''

class Controlling:

    def __init__(self):
        
        smach.State.__init__(self, 
                             outcomes=['reached','battery_full','battery_low'],
                             input_keys=['moving_counter_in'],
                             output_keys=['moving_counter_out'])

    def execute(self, userdata):

    	pass


class Recharge:

    def __init__(self):

        smach.State.__init__(self, 
                             outcomes=['battery_low','battery_full','reasoned'],
                             input_keys=['recharge_counter_in'],
                             output_keys=['recharge_counter_out'])

    def execute(self, userdata):

    	pass

'''


def main():

    rospy.init_node('fsm')

    interfacehelper = InterfaceHelper()

	# Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])
    sm.userdata.sm_counter = 0

    with sm:

    	smach.StateMachine.add('INITIAL', Initial(interfacehelper), 
                               transitions={'start':'REASONING', 
                                            'reasoned':'INITIAL'},
                               remapping={'initial_counter_in':'sm_counter', 
                                          'initial_counter_out':'sm_counter'})

    	smach.StateMachine.add('REASONING', Reasoning(interfacehelper), 
                               transitions={'start':'REASONING',
                                            'reasoned':'POINTING'},
                               remapping={'reasoning_counter_in':'sm_counter', 
                                          'reasoning_counter_out':'sm_counter'})

    
    	smach.StateMachine.add('POINTING', Pointing(interfacehelper), 
					    	   transitions={'pointed':'POINTING',
					    	   				'reasoned':'POINTING'},

					    	   remapping={'pointing_counter_in':'sm_counter', 
					    				  'pointing_counter_out':'sm_counter'})
    '''
    	smach.StateMachine.add('CONTROLLING', Controlling(), 
					    	   transitions={'reached':'REASONING',
								    	    'battery_low':'RECHARGE',
								    	    'battery_full':'CONTROLLING'},

					    	   remapping={'controlling_counter_in':'sm_counter', 
					    				  'controlling_counter_out':'sm_counter'})
    	
    '''

    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()