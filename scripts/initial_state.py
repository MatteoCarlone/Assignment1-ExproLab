#!/usr/bin/env python

import sys
import roslib
import rospy
import actionlib
from os.path import dirname, realpath

from std_srvs.srv import Empty , EmptyResponse

from armor_api.armor_client import ArmorClient

import time

from exprolab_1 import environment as env


class InitialState:

    def __init__(self):

        self.client = ArmorClient("armor_client", "reference")

        self.path = dirname(realpath(__file__))
        self.path = self.path + "/../topology/"

        rospy.Service(env.SERVER_START , Empty , self.execute)

    def execute(self,request):

        print('\n###############\nTOPOLOGY LOADING EXECUTION')
        
        curr_time = int(time.time())

        self.client.utils.load_ref_from_file(self.path + "topological_map.owl", "http://bnc/exp-rob-lab/2022-23",
                                            True, "PELLET", True, False)  
                                            # initializing with buffered manipulation and reasoning
        self.client.utils.mount_on_ref()
        self.client.utils.set_log_to_terminal(True)

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

        self.client.manipulation.add_objectprop_to_ind("isIn", 'Robot1', "E")

        for r in env.Loc:

            self.client.manipulation.add_dataprop_to_ind('visitedAt',r, 'Long', str(curr_time))


        self.client.call('DISJOINT','IND','',env.Loc)

        self.client.utils.apply_buffered_changes()
        self.client.utils.sync_buffered_reasoner()

        print('LOADING COMPLETED')

        return EmptyResponse()

if __name__ == "__main__":

    rospy.init_node(env.NODE_INIT_STATE, log_level=rospy.INFO)
    # Instantiate the node manager class and wait.
    InitialState()
    rospy.spin()