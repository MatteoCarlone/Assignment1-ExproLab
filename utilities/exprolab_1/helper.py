#!/usr/bin/env python

import rospy
from actionlib import SimpleActionClient
from threading import Lock

from exprolab_1.msg import PlanAction, ControlAction

from exprolab_1.srv import Start , StartRequest
from exprolab_1.srv import Reason , ReasonRequest


class ActionClientHelper:

	def __init__(self, service_name,action_type, done_cb = None , feedback_cb = None, mutex = None):

		self.reset_states()

		self._service_name = service_name

		if mutex is None:
			self.mutex = Lock()
		else:
			self._mutex = mutex

		self._client = SimpleActionClient(service_name,action_type)

		self._external_done_cb = done_cb

		self._external_feedback_cb = feedback_cb

		self._client.wait_for_server()

	def send_goal(self,goal):

		if not self._is_running:

			self._client.send_goal(goal,
				done_cb = self._done_cb,
				feedback_cb = self._feedback_cb)

			self._is_running = True
			self._is_done = False
			self._results = None

		else:

			print("Warning!!")

	def reset_states(self):

		self._is_running = False
		self._is_done = False
		self._results = None

	def cancel_goals(self):

		if(self._is_running):

			self._client.cancel_all_goals()
			self.reset_states()

		else:

			print("Warning!!")

	def _feedback_cb(self, feedback):
		self._mutex.acquire()
		try:
			if self._external_feedback_cb is not None:
				print(' '+ self._service_name + ' action server provide feedback: '+ str(feedback))
				self._external_feedback_cb(feedback)
		finally:
			self._mutex.release()

	def _done_cb(self, status , results):

		self._mutex.acquire()

		try:

			self._is_running = False
			self._is_done = True
			self._results = results

			if self._external_done_cb is not None:
				print(' '+ self._service_name + ' done with state: '+ 
					self._client.get_state_txt() + ' and result: ' + self._service_name)

				self._external_done_cb(status, result)

		finally:

			self._mutex.release()

	def is_done(self):
		return self._is_done

	def get_results(self):

		if self._is_done:

			return self._results

		else:

			print('Error!!')

	def is_running(self):
		return self._is_running




class InterfaceHelper:

	def __init__(self):

		self.mutex = Lock()
		self.reset_states()
		self._start = True

		self.planner_client = ActionClientHelper('motion/planner',PlanAction,mutex = self.mutex)

	def reset_states(self):

		self._start = False
		self._reason = False
		self._point = False

	def start_client(self,req):

		# wanting for the service to be online 
		rospy.wait_for_service('start')

		try:

			start_srv = rospy.ServiceProxy('start', Start)

			resp = StartRequest(req)

			result = start_srv(resp)

			self.mutex.acquire()

			try:

				if result is not None:

					self.reset_states()

					self._reason = True

			finally:

				self.mutex.release()

		except rospy.ServiceException as e:

			self.reset_states

			rospy.logerr("Exception occurred: %s", str(e))

	def reason_client(self,req):

		# wanting for the service to be online 
		rospy.wait_for_service('reason')

		try:

			reason_srv = rospy.ServiceProxy('reason', Reason)

			resp = ReasonRequest(req)

			result = reason_srv(resp)

			self.mutex.acquire()

			try:

				if result is not None:

					self.reset_states()

					self._point = True

					self.to_point = result.point 

			finally:

				self.mutex.release()

		except rospy.ServiceException as e:

			self.reset_states

			rospy.logerr("Exception occurred: %s", str(e))


	def should_reasoning_start(self):

		return self._reason

	def should_pointing_start(self):

		return self._point

	def room_togo(self):

		return self.to_point
















