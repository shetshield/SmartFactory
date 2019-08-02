#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import String

x = y = 0.0
tgt_x  = 0.15 # Move Forward
tgt_y  = 0.0  # Move Forward
number = 10

kp = 0.2
print("Press 0 ~ 9")
goal     = input()
goal_lst = [i for i in range(10)]
while goal not in goal_lst :
	print("Re-Press Between 0 ~ 9")
	goal = input()
print("Find the Goal number:" + str(goal))

def get_position(_pos_msg) :
	global x, y
	pos3d = pos_msg.pose.pose.position
	x = pos3d.x
	y = pos3d.y
	# print(x, y)

def get_number(_num_msg) :
	global number
	number = int(num_msg.data)
	print(number)

# Protocol: Node_Name = File_Name
rospy.init_node('mv_mobile')

sub_odom = rospy.Subscriber('/odom', Odometry, get_position)
sub_num  = rospy.Subscriber('chatter', String, get_number)
pub_vel  = rospy.Publisher('cmd_vel', Twist, queue_size=1)
# cmd = Twist()
cmd = Twist()
r   = rospy.Rate(5)

if number == goal :
	find = True
else :
	find = False

while not rospy.is_shutdown() :
	try :
		if find :
			cmd.linear.x = kp * (tgt_x - x)
			cmd.linear.y = kp * (tgt_y - y)
			pub_vel.publish(cmd)
			print("Found and go")
			if number != goal :
				find = False
		else :
			print(number, goal, number==goal)
			cmd.linear.x = 0
			cmd.linear.y = 0
			pub_vel.publish(cmd)
			print("Try to Find Goal")
			if number == goal :
				find = True

	except rospy.ROSInterruptException :
		cmd.linear.x = 0
		cmd.linear.y = 0
		pub_vel.publish(cmd)
		r.sleep()
		print("Error Occured")
		exit()
