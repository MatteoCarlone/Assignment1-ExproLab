#!/usr/bin/env python

import sys
import roslib
import rospy
import actionlib
from os.path import dirname, realpath

from exprolab_1.srv import Start , StartResponse

from armor_api.armor_client import ArmorClient


class InitialState:

    def __init__(self):

        self.client = ArmorClient("armor_client", "reference")

        self.path = dirname(realpath(__file__))
        self.path = self.path + "/../topology/"

        rospy.Service('start', Start , self.execute)

    def execute(self,request):

        if request.load is None:

            print('Request cannot be empty')

        else:

            print('Start Loading')

            self.client.utils.load_ref_from_file(self.path + "topological_map.owl", "http://bnc/exp-rob-lab/2022-23",
                                                True, "PELLET", True, False)  
                                                # initializing with buffered manipulation and reasoning
            self.client.utils.mount_on_ref()
            self.client.utils.set_log_to_terminal(True)

            self.client.manipulation.add_ind_to_class('robot', "Robot")

            self.client.manipulation.add_objectprop_to_ind("hasDoor", 'E', "D6")
            self.client.manipulation.add_objectprop_to_ind("hasDoor", 'E', "D7")

            self.client.manipulation.add_objectprop_to_ind("hasDoor", 'C1', "D1")
            self.client.manipulation.add_objectprop_to_ind("hasDoor", 'C1', "D2")
            self.client.manipulation.add_objectprop_to_ind("hasDoor", 'C1', "D5")
            self.client.manipulation.add_objectprop_to_ind("hasDoor", 'C1', "D6")

            self.client.manipulation.add_objectprop_to_ind("hasDoor", 'C2', "D3")
            self.client.manipulation.add_objectprop_to_ind("hasDoor", 'C2', "D4")
            self.client.manipulation.add_objectprop_to_ind("hasDoor", 'C2', "D5")
            self.client.manipulation.add_objectprop_to_ind("hasDoor", 'C2', "D7")

            self.client.manipulation.add_objectprop_to_ind("hasDoor", 'R1', "D1")
            self.client.manipulation.add_objectprop_to_ind("hasDoor", 'R2', "D2")
            self.client.manipulation.add_objectprop_to_ind("hasDoor", 'R3', "D3")
            self.client.manipulation.add_objectprop_to_ind("hasDoor", 'R4', "D4")

            self.client.manipulation.add_objectprop_to_ind("isIn", 'robot', "E")

            self.client.call('DISJOINT','IND','',['E','C1','C2','R1','R2','R3','R4','D1','D2','D3','D4','D5','D6','D7'])

            self.client.utils.apply_buffered_changes()
            self.client.utils.sync_buffered_reasoner()

            self.client.utils.save_ref_with_inferences(self.path + "test_disjoint.owl")

            print('sleep')
            rospy.sleep(5)
            print('send response')

            return StartResponse('approved')

if __name__ == "__main__":

    rospy.init_node('initial_state', log_level=rospy.DEBUG)
    # Instantiate the node manager class and wait.
    InitialState()
    rospy.spin()