#!/usr/bin/env python

import sys
import roslib
import rospy
import threading
import random

from std_msgs.msg import Bool

from exprolab_1.helper import InterfaceHelper

from exprolab_1.srv import Recharge , RechargeResponse


class Battery(object):
	
	def __init__(self):

		rospy.Service('recharge', Recharge , self.execute)

		interfacehelper = InterfaceHelper()
		self._helper = interfacehelper
		th = threading.Thread(target=self._random_notifier)
		th.start()

		self._battery_low = False

		self._random_battery_time = [30,50]

	def execute(self,request):

		# A List of Items
		items = list(range(0, 57))
		l = len(items)

		# Initial call to print 0% progress
		self._printProgressBar(0, l, prefix = 'Progress:', suffix = 'Complete', length = 30)

		for i, item in enumerate(items):

			rospy.sleep(0.1)

			self._printProgressBar(i + 1, l, prefix = 'Progress:', suffix = 'Complete', length = 30)

		return RechargeResponse('full')



	def _random_notifier(self):

		delay = 0

		publisher = rospy.Publisher('battery_low', Bool, queue_size=1, latch=True)

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

	rospy.init_node('battery', log_level=rospy.INFO)
	Battery()
	rospy.spin()

