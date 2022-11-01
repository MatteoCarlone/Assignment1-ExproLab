#!/usr/bin/env python

import rospy
from actionlib import SimpleActionClient
from threading import Lock

from exprolab_1 import environment as env
from exprolab_1.ActionHelper import ActionClientHelper

from armor_api.armor_client import ArmorClient


from std_msgs.msg import Bool
from std_srvs.srv import Empty
from exprolab_1.msg import PlanAction, PlanGoal, ControlAction, ControlGoal
from exprolab_1.srv import Reason , ReasonRequest

import re

class InterfaceHelper:

	def __init__(self):

		self.mutex = Lock()
		self.reset_states()
		self._start = True

		self.client = ArmorClient("armor_client", "reference")

		self.planner_client = ActionClientHelper(env.ACTION_PLANNER,PlanAction,mutex = self.mutex)
		self.controller_client = ActionClientHelper(env.ACTION_CONTROLLER,ControlAction,mutex = self.mutex)

		self.sub_battery = rospy.Subscriber(env.TOPIC_BATTERY_LOW, Bool, self._battery_cb)

	def reset_states(self):

		self._start = False
		self._reason = False
		self._point = False
		self._battery_low = False
		self._battery_full = False

	def start_client(self):

		# wanting for the service to be online 
		rospy.wait_for_service(env.SERVER_START)

		try:

			start_srv = rospy.ServiceProxy(env.SERVER_START, Empty)
			resp = start_srv()

			self.mutex.acquire()

			try:

				self.reset_states()

				self._reason = True

			finally:

				self.mutex.release()

		except rospy.ServiceException as e:

			self.reset_states()

			rospy.logerr("Exception occurred: %s", str(e))

	def reason_client(self):

		# wanting for the service to be online 
		rospy.wait_for_service(env.SERVER_REASON)

		try:

			reason_srv = rospy.ServiceProxy(env.SERVER_REASON, Reason)

			result = reason_srv()

			self.mutex.acquire()

			try:

				if result is not None:

					self.reset_states()

					self._point = True

					self.to_point = result.point 

			finally:

				self.mutex.release()

		except rospy.ServiceException as e:

			self.reset_states()

			rospy.logerr("Exception occurred: %s", str(e))

	def recharge_client(self):

		# wanting for the service to be online 
		rospy.wait_for_service(env.SERVER_RECHARGE)

		try:

			recharge_srv = rospy.ServiceProxy(env.SERVER_RECHARGE, Empty)

			result = recharge_srv()

			self.mutex.acquire()

			try:

				if result is not None:

					self.reset_states()

					self._battery_full = True

			finally:

				self.mutex.release()

		except rospy.ServiceException as e:

			self.reset_states()

			rospy.logerr("Exception occurred: %s", str(e))


	def _battery_cb(self,battery_value):

		self._battery_low = battery_value.data


	def send_planner_goal(self,low):

		self.reason()

		can_reach = self.client.query.objectprop_b2_ind('canReach','Robot1')

		can_reach = self.list_formatter(can_reach,'#','>')

		if low and env.START_LOC in can_reach:

			self.to_point = env.START_LOC
			
			goal = PlanGoal(target= self.to_point)

			self.planner_client.send_goal(goal)

		else:

			if self.to_point is not None:

				goal = PlanGoal(target= self.to_point)

				self.planner_client.send_goal(goal)

			else:

				print('PlanGoal Error')

	def send_controller_goal(self):

		path = self.planner_client.get_results()

		if path.via_points is not None:

			goal = ControlGoal(point_set = path.via_points)

			self.controller_client.send_goal(goal)

		else:

			print('ControlGoal Error')

	def is_battery_full(self):

		return self._battery_full

	def is_battery_low(self):

		return self._battery_low

	def should_reasoning_start(self):

		return self._reason

	def should_pointing_start(self):

		return self._point


	def reason(self):

		self.client.utils.apply_buffered_changes()
		self.client.utils.sync_buffered_reasoner()

	def list_formatter(self,raw_list,start,end):

		formatted_list = [re.search(start+'(.+?)'+end,string).group(1) for string in raw_list]

		return formatted_list

















