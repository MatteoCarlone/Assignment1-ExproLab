#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import time
import string

from exprolab_1.helper import InterfaceHelper


class Initial(smach.State):

    def __init__(self,interfacehelper):

        smach.State.__init__(self, 
                             outcomes=['start','reasoned'],
                             input_keys=['initial_counter_in'],
                             output_keys=['initial_counter_out'])

        self._helper = interfacehelper

    def execute(self, userdata):

        self._helper.start_client('load') 

        while not rospy.is_shutdown():

            # acquire mutex

            self._helper.mutex.acquire()

            try:

    			# look for transition flags

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

        self._helper.reason_client('reason')

        while not rospy.is_shutdown():

            # acquire mutex

            self._helper.mutex.acquire()

            try:

                # look for transition flags

                if self._helper.should_pointing_start():

                    self._helper.reset_states()

                    print('reset flags and transition')

                    return 'reasoned'

            finally:

                # release mutex

                self._helper.mutex.release()

            rospy.sleep(0.3)


class Pointing(smach.State):

    def __init__(self,interfacehelper):
        
        smach.State.__init__(self, 
                             outcomes=['pointed','reasoned','battery_low'],
                             input_keys=['moving_counter_in'],
                             output_keys=['moving_counter_out'])

        self._helper = interfacehelper

    def execute(self, userdata):

        self._helper.send_planner_goal()

        while not rospy.is_shutdown():

            # acquire mutex

            self._helper.mutex.acquire()

            try:

                # look for transition flags

                if self._helper.is_battery_low():

                    self._helper.reset_states()

                    return 'battery_low'

                if self._helper.planner_client.is_done():

                    self._helper.reset_states()

                    return 'pointed'

            finally:

                # release mutex

                self._helper.mutex.release()

            rospy.sleep(0.3)

class Controlling(smach.State):

    def __init__(self,interfacehelper):
        
        smach.State.__init__(self, 
                             outcomes=['reached','battery_low'],
                             input_keys=['moving_counter_in'],
                             output_keys=['moving_counter_out'])

        self._helper = interfacehelper

    def execute(self, userdata):

        self._helper.send_controller_goal()

        while not rospy.is_shutdown():

            # acquire mutex

            self._helper.mutex.acquire()

            try:

                # look for transition flags
                if self._helper.is_battery_low():

                    self._helper.reset_states()

                    return 'battery_low'

                if self._helper.controller_client.is_done():

                    self._helper.reset_states()

                    return 'reached'

            finally:

                # release mutex

                self._helper.mutex.release()

            rospy.sleep(0.3)

class Recharge(smach.State):

    def __init__(self,interfacehelper):

        smach.State.__init__(self, 
                             outcomes=['battery_low','battery_full'],
                             input_keys=['recharge_counter_in'],
                             output_keys=['recharge_counter_out'])

        self._helper = interfacehelper

    def execute(self, userdata):

        self._helper.recharge_client('recharge')

        while not rospy.is_shutdown():

            # acquire mutex
            self._helper.mutex.acquire()

            try:

                # look for transition flags
                print('before low check')

                if self._helper.is_battery_low():

                    print('battery low check')

                    self._helper.reset_states()

                    return 'battery_low'

                print('before full check')

                if self._helper.is_battery_full():

                    print('battery full check')

                    self._helper.reset_states()

                    return 'battery_full'

            finally:

                # release mutex

                self._helper.mutex.release()

            rospy.sleep(0.3)


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
					    	   transitions={'pointed':'CONTROLLING',
                                            'reasoned':'POINTING',
                                            'battery_low':'RECHARGE'},

					    	   remapping={'pointing_counter_in':'sm_counter', 
                                          'pointing_counter_out':'sm_counter'})
    
        smach.StateMachine.add('CONTROLLING', Controlling(interfacehelper), 
					    	   transitions={'reached':'REASONING',
                                            'battery_low':'RECHARGE'},

					    	   remapping={'controlling_counter_in':'sm_counter', 
					    				  'controlling_counter_out':'sm_counter'})
        smach.StateMachine.add('RECHARGE', Recharge(interfacehelper), 
                               transitions={'battery_full':'REASONING',
                                            'battery_low':'RECHARGE'},

                               remapping={'recharge_counter_in':'sm_counter', 
                                          'recharge_counter_out':'sm_counter'})


    
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