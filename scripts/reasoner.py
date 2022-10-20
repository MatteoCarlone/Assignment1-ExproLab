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

class Reasoner:

   def __init__(self):

      rospy.Service('reason', Reason , self.execute)

      self.client = ArmorClient("armor_client", "reference")
      self.reachable_list = []


   def execute(self,request):

      if request.reason is None:

            print('Request cannot be empty')

      else:

         print('#### REASONING ####')

         can_reach = self.client.query.objectprop_b2_ind('canReach','robot')

         for i in can_reach:

            self.reachable_list.append(re.search('#(.+?)>',i).group(1))


         print('\nthe robot can reach location: ', self.reachable_list[0])

         print('\nthe robot can reach location: ', self.reachable_list[1])

         self.client.call('REASON','','','')

         #print(self.client.query.ind_b2_class('URGENT'))

         isin = self.client.query.objectprop_b2_ind('isIn','robot')

         print(re.search('#(.+?)>',isin[0]).group(1))

         to_point = random.choice(self.reachable_list)

         print(env.Map[to_point])

         return ReasonResponse(to_point)

def main():

	# Initialise the node  
    rospy.init_node('reasoner', log_level=rospy.INFO)

    Reasoner()

    rospy.spin()


if __name__ == '__main__':

   main()