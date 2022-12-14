#!/usr/bin/env python

"""
.. module:: initial_state
    :platform: Unix
    :synopsis: Python code to initialize and load the topology

.. moduleauthor:: Matteo Carlone <matteo.carlone99@gmail.com>

Service:
    /start 
Action:
    /armor_client

This Node start the whole program by initializing and loading the topology.

"""

#---Libraries---#

import sys
import roslib
import rospy
import actionlib
import time
from os.path import dirname, realpath

from std_srvs.srv import Empty , EmptyResponse
from armor_api.armor_client import ArmorClient

from exprolab_1 import environment as env

#--------------#

class InitialState:

    """
    Class representing the Initial-State of the Smach-State-Machine, which load the 
    Ontology initialized with the environment description, and starting point of the Robot.
    ...

    Methods
    ----------

    __init__(self)

        Initialization of parameters:

            client:ros_action_client
                Armor-Client to set-up the Ontology
            path:str
                The Ontology folder-path to load 
            server:ros_server
                the server of the Empty service /start with Callback execute()

    execute(self,request)

        Server Callback of the /start service requested from the fsm module to start up 
        the whole program.

        This Callback-Server load the ontology, set all the room of the environment with the respective doors,
        the starting position of the robot, already initialized in the starting ontology, the disjoint function
        that let the reasoner understanding all the individuals and I assumed all the rooms visited at time zero 
        in order to yet start the counter for the URGENT rooms.

    """

    def __init__(self):

        # armor - client
        self.client = ArmorClient("armor_client", "reference")

        # absolute ontology path
        self.path = dirname(realpath(__file__))
        self.path = self.path + "/../topology/"

        # /start Empty Server , execute callback
        self.server = rospy.Service(env.SERVER_START , Empty , self.execute)

    def execute(self,request):

        print('\n###############\nTOPOLOGY LOADING EXECUTION')
        
        # get the current time instant 
        curr_time = int(time.time())

        # load ontology from the absolute path 
        self.client.utils.load_ref_from_file(self.path + "topological_map.owl", "http://bnc/exp-rob-lab/2022-23",
                                            True, "PELLET", True, False)  
                                            
        self.client.utils.mount_on_ref()
        self.client.utils.set_log_to_terminal(True)

        # -------  Set up all the rooms with respective doors ------- #

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

        # ----------------------------------------------------------- #

        # Robot starting room
        self.client.manipulation.add_objectprop_to_ind("isIn", 'Robot1', "E")

        # Set all rooms visited at curr_time time instant
        for room in env.Loc:

            self.client.manipulation.add_dataprop_to_ind('visitedAt',room, 'Long', str(curr_time))


        # Disjoint for Individuals understanding
        self.client.call('DISJOINT','IND','',env.Loc)

        # First Reasoning
        self.client.utils.apply_buffered_changes()
        self.client.utils.sync_buffered_reasoner()

        print('LOADING COMPLETED')

        # returning an empty response to notify the completed load of the ontology
        return EmptyResponse()

if __name__ == "__main__":

    # Initialize the ROS-Node
    rospy.init_node(env.NODE_INIT_STATE, log_level=rospy.INFO)

    # Instantiate the node manager class and wait.
    InitialState()

    rospy.spin()