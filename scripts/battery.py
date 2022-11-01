#!/usr/bin/env python

import sys
import roslib
import rospy
import threading
import random

from std_msgs.msg import Bool

from exprolab_1.helper import InterfaceHelper

from std_srvs.srv import Empty , EmptyResponse

from exprolab_1 import environment as env


class Battery(object):
	
	def __init__(self):

		rospy.Service(env.SERVER_RECHARGE, Empty , self.execute)

		interfacehelper = InterfaceHelper()
		self._helper = interfacehelper

		self._battery_low = False

		self._random_battery_time = rospy.get_param(env.RND_BATTERY_TIME)

		th = threading.Thread(target=self._random_notifier)
		th.start()

	def execute(self,request):

		print('\n###############\nRECHARGE EXECUTION')

		isin = self._helper.client.query.objectprop_b2_ind('isIn','Robot1')

		isin = self._helper.list_formatter(isin,'#','>')

		print('The robot docked for recharge in: ' + isin[0] + '\n')

		# A List of Items
		items = list(range(0, 57))
		l = len(items)

		# Initial call to print 0% progress
		self._printProgressBar(0, l, prefix = 'Progress:', suffix = 'Complete', length = 30)

		for i, item in enumerate(items):

			rospy.sleep(0.1)

			self._printProgressBar(i + 1, l, prefix = 'Progress:', suffix = 'Complete', length = 30)

		print('BATTERY FULL')

		return EmptyResponse()


	def _random_notifier(self):

		delay = 0

		publisher = rospy.Publisher(env.TOPIC_BATTERY_LOW, Bool, queue_size=1, latch=True)

		while not rospy.is_shutdown():
			
			# Wait for simulate battery usage.
			delay = random.uniform(self._random_battery_time[0], self._random_battery_time[1])
			rospy.sleep(delay)
			# Change battery state.
			self._battery_low = True
			# Publish battery level.
			publisher.publish(Bool(self._battery_low))

	# Print iterations progress
	def _printProgressBar (self,iteration, total, prefix = '', suffix = '', decimals = 1, length = 100, fill = '¦', printEnd = "\r"):
	    """
	    Call in a loop to create terminal progress bar
	    @params:
	        iteration   - Required  : current iteration (Int)
	        total       - Required  : total iterations (Int)
	        prefix      - Optional  : prefix string (Str)
	        suffix      - Optional  : suffix string (Str)
	        decimals    - Optional  : positive number of decimals in percent complete (Int)
	        length      - Optional  : character length of bar (Int)
	        fill        - Optional  : bar fill character (Str)
	        printEnd    - Optional  : end character (e.g. "\r", "\r\n") (Str)
	    """
	    percent = ("{0:." + str(decimals) + "f}").format(100 * (iteration / float(total)))
	    filledLength = int(length * iteration // total)
	    bar = fill * filledLength + '-' * (length - filledLength)
	    print(f'\r{prefix} |{bar}| {percent}% {suffix}', end = printEnd)
	    # Print New Line on Complete
	    if iteration == total: 
	        print()


if __name__ == '__main__':

	rospy.init_node(env.NODE_BATTERY, log_level=rospy.INFO)
	Battery()
	rospy.spin()

