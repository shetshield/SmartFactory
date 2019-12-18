#!/usr/bin/env python

import rospy, sys, moveit_commander, time, moveit_msgs.msg
from geometry_msgs.msg import Pose
from copy import deepcopy
from math import pi
from moveit_commander.conversions import pose_to_list

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('moveit_ur3', anonymous=False)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = 'manipulator' # or 'endeffector' for ur3
group  = moveit_commander.MoveGroupCommander(group_name)
group.set_max_acceleration_scaling_factor(0.4) # Limit Acceleration

# Uncomment below if visualization is needed in RVIZ
# display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

init_joint_state = [-1.4873150030719202, -1.5659659544574183, -2.1959059874164026, -0.9785626570331019, 1.5747051239013672, 3*pi/4]

def go_to_joint_initial_joint_state():
    print(".... Initializing")
    joint_goal = group.get_current_joint_values()
    print joint_goal

    group.go(init_joint_state, wait=True)
    time.sleep(3)
    group.stop()
    print("\n")
    for i in range(5) :
        print(".....................")
    print("Initialize is done...\n")

def go_to_pose_goal():
    pose_goal = group.get_current_pose().pose
    group.set_planning_time(5)
    print(pose_goal)
    # pose_goal.orientation.w = pose_goal.orientation.w
    pose_goal.position.x = pose_goal.position.x + 0.1
    # pose_goal.position.y = pose_goal.position.y
    # pose_goal.position.z = pose_goal.position.z

    group.set_pose_target(pose_goal)
    group.go(pose_goal, wait=True)
    print(pose_goal)
    # pose_goal can be list of 6 elements(x, y, z, Rx, Ry, Rz) or list of 7 elements(x, y, z, Quaternion)
    # 6 elemetns example(7 elements example is presented above)
    # pose_goal = [pose_goal.position.x, pose_goal.position.y, pose_goal.position.z, 0, pi/2, pi/4]
    # group.set_pose_target(pose_goal)
    # group.go(pose_goal, wait=True)

    time.sleep(3)
    group.stop()
    group.clear_pose_targets()

def main():
    go_to_joint_initial_joint_state() # Initialize
    go_to_pose_goal()                 # Go-to Task position

if __name__ == '__main__':
    main()
