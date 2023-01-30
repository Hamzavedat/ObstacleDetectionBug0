#! /usr/bin/env python

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
import threading
import math
from tf import transformations

obstacle = 'no'

initial_position = Point()
position_ = Point()
target_position = Point()
yaw_ = 0
state_robot = 0
state_ = 0
state_wall = 0
yaw_precision_ = math.pi/90 # +/- 2 degree allowed
dist_precision_ = 0.1
pub = None
delay = 0
count = 0

def clbk_obs(msg):

	global obstacle, state_wall
	obstacle = msg.data
	
def change_follow():
	global obstacle, state_wall
	while True:
		if(obstacle == 'no'):
			state_wall = 0
		elif(obstacle == 'm'):
			state_wall = 1
		elif(obstacle == 'r'):
			state_wall = 2
		elif(obstacle == 'l'):
			state_wall = 0
		elif(obstacle == 'rlm'):
			state_wall = 1
		elif(obstacle == 'rm'):
			state_wall = 1
		elif(obstacle == 'lm'):
			state_wall = 1
		elif(obstacle == 'rl'):
			state_wall = 0


def clbk_odom(msg):
	global position_, yaw_

	# position
	position_ = msg.pose.pose.position

	# yaw
	quaternion = (
		msg.pose.pose.orientation.x,
		msg.pose.pose.orientation.y,
		msg.pose.pose.orientation.z,
		msg.pose.pose.orientation.w)
	euler = transformations.euler_from_quaternion(quaternion)
	yaw_ = euler[2]

def distance_to_line(p0):
	# p0 is the current position
	# p1 and p2 points define the line
	global initial_position , target_position
	p1 = initial_position
	p2 = target_position
	# here goes the equation
	up_eq = math.fabs((p2.y - p1.y) * p0.x - (p2.x - p1.x) * p0.y + (p2.x * p1.y) - (p2.y * p1.x))
	lo_eq = math.sqrt(pow(p2.y - p1.y, 2) + pow(p2.x - p1.x, 2))
	distance = up_eq / lo_eq

	return distance

def normalize_angle(angle):
	if(math.fabs(angle) > math.pi):
		angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
	return angle

def fix_yaw(des_pos):
	global yaw_, yaw_precision_, state_
	desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
	err_yaw = normalize_angle(desired_yaw - yaw_)

	rospy.loginfo(err_yaw)

	twist_msg = Twist()
	if math.fabs(err_yaw) > yaw_precision_:
		twist_msg.angular.z = 0.7 if err_yaw > 0 else -0.7

	pub.publish(twist_msg)

	# state change conditions
	if math.fabs(err_yaw) <= yaw_precision_:
		print 'Yaw error: [%s]' % err_yaw
		state_ = 1

def go_straight_ahead(des_pos):
	global yaw_, pub, yaw_precision_, state_, position_
	desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
	err_yaw = desired_yaw - yaw_
	err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))

	if err_pos > dist_precision_:
		twist_msg = Twist()
		twist_msg.linear.x = 0.6
		twist_msg.angular.z = 0.2 if err_yaw > 0 else -0.2
		pub.publish(twist_msg)
	else:
		print 'Position error: [%s]' % err_pos
		state_ = 2

	# state change conditions
	if math.fabs(err_yaw) > yaw_precision_:
		print 'Yaw error: [%s]' % err_yaw
		state_ = 0

def done():
	twist_msg = Twist()
	twist_msg.linear.x = 0
	twist_msg.angular.z = 0
	pub.publish(twist_msg)
	print 'Target reached'

def find_wall():
	msg = Twist()
	msg.linear.x = 0.3
	msg.angular.z = -0.2
	return msg

def turn_left():
	msg = Twist()
	msg.angular.z = 0.3
	return msg

def follow_the_wall():
	msg = Twist()
	msg.linear.x = 0.5
	return msg

def go_to_point():
	global state_robot, state_, target_position

	if state_ == 0:
		fix_yaw(target_position)
	elif state_ == 1:
		go_straight_ahead(target_position)
	elif state_ == 2:
		done()
	
	
def follow_wall():
	global state_wall	    
	msg = Twist()
	if state_wall == 0:
		msg = find_wall()
	elif state_wall == 1:
		msg = turn_left()
	elif state_wall == 2:
		msg = follow_the_wall()
	pub.publish(msg)

		

def main():

	global pub, target_position, state_robot, obstacle, position, initial_position, delay, count

	target_position.x = input('x position:')
	target_position.y = input('y position:')
	target_position.z = 0
	rospy.init_node('BUG2')
	rospy.Subscriber('/info', String, clbk_obs)
	rospy.Subscriber('/odom', Odometry, clbk_odom)
	
	initial_position = position_

	pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)




	rate = rospy.Rate(20)
	while not rospy.is_shutdown():    
		distance_position_to_line = distance_to_line(position_)
		if state_robot == 0:
			go_to_point()
			if obstacle != 'no':
				state_robot = 1
				delay = 0
    
		elif state_robot == 1:
			follow_wall()
			if distance_position_to_line < 0.5 and delay > 5:
				state_robot = 0
				delay = 0

		rate.sleep()
		count = count + 1
		if count == 20:
			delay = delay + 1
			count = 0

if __name__ == '__main__':
	thr_follow = threading.Thread(target=change_follow)
	thr_follow.start()
	main()
