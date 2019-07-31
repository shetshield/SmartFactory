#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

x = y = 0.0
tgt_x = 0.2
tgt_y = 0.2

kp = 0.5

def get_position(msg) :
	global x, y
	pos3d = msg.pose.pose.position
	x = pos3d.x
	y = pos3d.y
	# print(x, y)

# Protocol: Node_Name = File_Name
rospy.init_node('mv_mobile')

sub = rospy.Subscriber('/odom', Odometry, get_position)
pub = rospy.Publisher('cmd_vel', Point, queue_size=1)
# cmd = Twist()
cmd = Point()
r   = rospy.Rate(10)

while not rospy.is_shutdown() :
	cmd_x = kp * (tgt_x - x)
	cmd_y = kp * (tgt_y - y)
	cmd.x = cmd_x
	cmd.y = cmd_y
	# cmd.linear.x = cmd_x
	# cmd.linear.y = cmd_y
	pub.publish(cmd)
	print("target_x={} current_x={} target_y={} current_y", tgt_x, x, tgt_y, y)
	r.sleep()
