#!/usr/bin/env python
import rospy, math
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
from std_msgs.msg import String

roll = pitch = yaw = 0.0
rot_angle = 360.

number = 10
print("Press 0 ~ 9")
goal     = input()
goal_lst = [i for i in range(10)]
while goal not in goal_lst :
	print("Re-Press 0 ~ 9")
	goal = input()
print("Find the Goal Number:"+str(goal))

kp = 0.01

def get_rotation(odom_msg) :
	global roll, pitch, yaw
	rot_q    = odom_msg.pose.pose.orientation
	rot_list = [rot_q.x, rot_q.y, rot_q.z, rot_q.w]
	(roll, pitch, yaw) = euler_from_quaternion(rot_list)
	# print(roll, pitch, yaw)

def num_check(num_msg) :
	global number
	number = int(num_msg.data)
	# print(number)

# Protocol: Node_Name = File_Name
rospy.init_node('rot_mobile')

Odom_sub = rospy.Subscriber('/odom', Odometry, get_rotation)
Chat_sub = rospy.Subscriber('chatter', String, num_check)
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
cmd = Twist()
r   = rospy.Rate(5)

if number == goal :
	find = True
else :
	find = False
while not rospy.is_shutdown() :
	try :
		if find :
			# Case of Found the goal
			cmd.angular.z = 0
			pub.publish(cmd)
			print("Current Heading:" +str(round(180./math.pi*yaw, 2))+" " + "number:" +str(number))
		else :
			# Case of not Found
			target_rad    = rot_angle * math.pi/180.0
			cmd.angular.z = kp * (target_rad - yaw)
			pub.publish(cmd)
			print("Current Heading:" + str(round(180./math.pi*yaw, 2))+ " " + "number:"+str(number))
			if number == goal :
				print("Find the Goal, stop rotating")
				find = True
				cmd.angular.z = 0.
				pub.publish(cmd)
				# Manipulator Operation Msg
				# Mobile Robot Operation Msg
				# e.g. Move Forward
		r.sleep()
	except rospy.ROSInterruptException :
		cmd.angular.z = 0.
		pub.publish(cmd)
		r.sleep()
		print("Shutdown")
		exit()
