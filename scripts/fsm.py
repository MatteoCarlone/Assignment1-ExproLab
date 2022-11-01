#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import time
import string

from exprolab_1.helper import InterfaceHelper

from exprolab_1 import environment as env

low = False

class Initial(smach.State):

    def __init__(self,interfacehelper):

        smach.State.__init__(self, 
                             outcomes=['start','reasoned'],
                             input_keys=['initial_counter_in'],
                             output_keys=['initial_counter_out'])

        self._helper = interfacehelper

    def execute(self, userdata):

        self._helper.start_client() 

        while not rospy.is_shutdown():

            # acquire mutex

            self._helper.mutex.acquire()

            try:

    			# look for transition flags

                if self._helper.should_reasoning_start():

                    self._helper.reset_states()

                    return 'start'

            finally:

    			# release mutex

                self._helper.mutex.release()

            rospy.sleep(0.3)


class Reasoning(smach.State):

    def __init__(self,interfacehelper):
        
        smach.State.__init__(self, 
                             outcomes=['start','reasoned','battery_low'],
                             input_keys=['reasoning_counter_in'],
                             output_keys=['reasoning_counter_out'])

        self._helper = interfacehelper

    def execute(self, userdata):

        self._helper.reason_client()

        while not rospy.is_shutdown():

            # acquire mutex

            self._helper.mutex.acquire()

            try:

                # look for transition flags
                if self._helper.is_battery_low():

                    self._helper.reset_states()

                    low = True

                    return 'battery_low'

                if self._helper.should_pointing_start():

                    self._helper.reset_states()

                    return 'reasoned'

            finally:

                # release mutex

                self._helper.mutex.release()

            rospy.sleep(0.3)


class Pointing(smach.State):

    def __init__(self,interfacehelper):
        
        smach.State.__init__(self, 
                             outcomes=['pointed','battery_low'],
                             input_keys=['pointing_counter_in'],
                             output_keys=['pointing_counter_out'])

        self._helper = interfacehelper

    def execute(self, userdata):

        global low

        self._helper.send_planner_goal(low)

        while not rospy.is_shutdown():

            # acquire mutex

            self._helper.mutex.acquire()

            try:

                # look for transition flags

                if self._helper.is_battery_low():

                    self._helper.reset_states()

                    self._helper.planner_client.cancel_goals()

                    low = True

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
                             input_keys=['controlling_counter_in'],
                             output_keys=['controlling_counter_out'])

        self._helper = interfacehelper

    def execute(self, userdata):

        global low

        self._helper.send_controller_goal()

        while not rospy.is_shutdown():

            # acquire mutex

            self._helper.mutex.acquire()

            try:

                # look for transition flags
                if self._helper.is_battery_low():

                    self._helper.reset_states()

                    self._helper.controller_client.cancel_goals()

                    low = True

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
                             outcomes=['battery_full','not_at_dock'],
                             input_keys=['recharge_counter_in'],
                             output_keys=['recharge_counter_out'])

        self._helper = interfacehelper

    def execute(self, userdata):

        global low

        self._helper.reason()

        isin = self._helper.client.query.objectprop_b2_ind('isIn','Robot1')

        isin = self._helper.list_formatter(isin,'#','>')

        if env.START_LOC not in isin:

            print('\nROBOT in: ',isin)
            print('Not in START LOCATION, Cannot DOC\n')

            return 'not_at_dock'

        self._helper.recharge_client()

        while not rospy.is_shutdown():

            # acquire mutex
            self._helper.mutex.acquire()

            try:

                if self._helper.is_battery_full():

                    low = False

                    self._helper.reset_states()

                    return 'battery_full'

            finally:

                # release mutex

                self._helper.mutex.release()

            rospy.sleep(0.3)


def main():

    rospy.init_node(env.NODE_FSM)

    interfacehelper = InterfaceHelper()

	# Create a SMACH state machine
    sm_top = smach.StateMachine(outcomes=['fsm'])
    sm_top.userdata.sm_counter = 0

    with sm_top:

        smach.StateMachine.add('INITIAL', Initial(interfacehelper), 
                               transitions={'start':'REASONING', 
                                            'reasoned':'INITIAL'},
                               remapping={'initial_counter_in':'sm_counter', 
                                          'initial_counter_out':'sm_counter'})

        smach.StateMachine.add('REASONING', Reasoning(interfacehelper), 
                               transitions={'start':'REASONING',
                                            'reasoned':'MOVING',
                                            'battery_low':'RECHARGE'},
                               remapping={'reasoning_counter_in':'sm_counter', 
                                          'reasoning_counter_out':'sm_counter'})

        sm_sub = smach.StateMachine(outcomes=['exit','low'])
        sm_sub.userdata.sm_counter = 0

        with sm_sub:
    
            smach.StateMachine.add('POINTING', Pointing(interfacehelper), 
    					    	   transitions={'pointed':'CONTROLLING',
                                                'battery_low':'low'},

    					    	   remapping={'pointing_counter_in':'sm_counter', 
                                              'pointing_counter_out':'sm_counter'})
        
            smach.StateMachine.add('CONTROLLING', Controlling(interfacehelper), 
    					    	   transitions={'reached':'exit',
                                                'battery_low':'low'},

    					    	   remapping={'controlling_counter_in':'sm_counter', 
    					    				  'controlling_counter_out':'sm_counter'})

        smach.StateMachine.add('MOVING', sm_sub, 
                               transitions={'exit':'REASONING',
                                            'low':'RECHARGE'},

                               remapping={'moving_counter_in':'sm_counter', 
                                          'moving_counter_out':'sm_counter'})   


        sm_recharge = smach.StateMachine(outcomes=['full'])
        sm_recharge.userdata.sm_counter = 0

        with sm_recharge:

            smach.StateMachine.add('RECH_BAR', Recharge(interfacehelper), 
                                   transitions={'battery_full':'full',
                                                'not_at_dock':'MOVE_TO_DOCK'},

                                   remapping={'bar_counter_in':'sm_counter', 
                                              'bar_counter_out':'sm_counter'})

            smach.StateMachine.add('MOVE_TO_DOCK', sm_sub, 
                                   transitions={'exit':'RECH_BAR',
                                                'low':'MOVE_TO_DOCK'},

                                   remapping={'moving_counter_in':'sm_counter', 
                                              'moving_counter_out':'sm_counter'}) 

        smach.StateMachine.add('RECHARGE', sm_recharge, 
                               transitions={'full':'REASONING'},

                               remapping={'recharge_counter_in':'sm_counter', 
                                          'recharge_counter_out':'sm_counter'})


    
    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm_top, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm_top.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()