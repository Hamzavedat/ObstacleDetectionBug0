#! /usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

regions = [10,10,10]

def clbk_laser(msg):
	#640 / 3 = 213
	global regions
	regions = [
        min(min(msg.ranges[0:212]), 10),
        min(min(msg.ranges[213:425]), 10),
        min(min(msg.ranges[426:639]), 10)
	]




if __name__ == '__main__':
	rospy.init_node('reading_laser')

	sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)

	pub = rospy.Publisher('/info', String, queue_size=10)

	message = String()

	while not rospy.is_shutdown():

		

		if(regions[0] < 1.5 and regions[1] < 1.5 and regions[2] < 1.5):
			message.data = 'rlm'

		elif(regions[0] < 1.5 and regions[1] < 1.5):
			message.data = 'rm'

		elif(regions[0] < 1.5 and regions[2] < 1.5):
			message.data = 'rl'

		elif(regions[0] < 1.5):
			message.data = 'r'

		elif(regions[1] < 1.5 and regions[2] < 1.5):
			message.data = 'lm'

		elif(regions[1] < 1.5):
			message.data = 'm'

		elif(regions[2] < 1.5):
			message.data = 'l'

		else:
			message.data = 'no'

		pub.publish(message)