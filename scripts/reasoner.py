#! /usr/bin/env python

import rospy
import roslib
import rospy
import actionlib
import random
import re

from exprolab_1 import environment as env

from armor_api.armor_client import ArmorClient

from exprolab_1.srv import Reason , ReasonResponse

from exprolab_1.helper import InterfaceHelper

import re

class Reasoner:

   def __init__(self):

      rospy.Service('reason', Reason , self.execute)

      self.client = ArmorClient("armor_client", "reference")

      interfacehelper = InterfaceHelper()
      self._helper = interfacehelper

      self.reachable_list = []
      self.urgent_list = []
      self.corridors = []

   def execute(self,request):

      if request.reason is None:
            print('Request cannot be empty')

      else:

         print('#### REASONING ####')

         self._helper.reason()

         isin = self.client.query.objectprop_b2_ind('isIn','Robot1')
         now = self.client.query.dataprop_b2_ind('now','Robot1')

         print('now the robot is in '+ re.search('#(.+?)>',isin[0]).group(1))
         print('Now timestamp: '+ re.search('"(.+?)"',str(now)).group(1))

         can_reach = self.client.query.objectprop_b2_ind('canReach','Robot1')
         urgent_list = self.client.query.ind_b2_class('URGENT')
         corridors = self.client.query.ind_b2_class('CORRIDOR')

         self.reachable_list = self._helper.list_formatter(can_reach,'#','>')
         self.urgent_list = self._helper.list_formatter(urgent_list,'#','>')
         self.corridors = self._helper.list_formatter(corridors,'#','>')

         print('The robot can reach Location: \n',self.reachable_list)


         room_to_go = self._next_room()


         visited_at = self.client.query.dataprop_b2_ind('visitedAt',room_to_go)
         visited_at = re.search('"(.+?)"',str(visited_at)).group(1)

         print('pointing location '+ room_to_go + ' lastly visited at time: ' + str(visited_at))

         self.reachable_list = []

         return ReasonResponse(room_to_go)

   def _next_room(self):

      RU_room = [i for i in self.reachable_list if i in self.urgent_list]
      
      if not RU_room:

         RC_room = [i for i in self.reachable_list if i in self.corridors]
         print(RC_room)

         if not RC_room:
            to_point = random.choice(self.reachable_list) 

         else:
            to_point = random.choice(RC_room)
      else:
         print(RU_room)
         to_point = random.choice(RU_room)

      return to_point


def main():

   # Initialise the node  
   rospy.init_node('reasoner', log_level=rospy.INFO)
   Reasoner()
   rospy.spin()

if __name__ == '__main__':

   main()