# smf_robot_control ROS package uploaded - 191218
# a.Environment
# 1) ubuntu 16.04.6
# 2) ROS : kinetic kame
# 3) UR3 software : 3.12(Require > 3.7)
# 4) UR ROS pakcage : ur_robot_driver

# Error MSG
# Unable to identify and set of controllers that can actuate ~
# open referenced "controllers.yaml"
# Edit action_ns "follow_joint_trajectory" to "scaled_pos_traj_controller/follow_joint_trajectory"
# controllers.yaml file is located in /ROS_PACKAGE_DIRECTORY/src/fmauch_universal_robot/ur3_moveit_config/config/controllers.yaml
# ex) ~/catkin_ws/src/fmauch_universal_robot/ur3_moveit_config/config/controllers.yaml

# Network Setup
# UR3 IP Address : 192.168.0.9
# HOST IP Address setting : 192.168.0.10 / 255.255.255.0 / 192.168.0.1

# Execution Sequence
# On UR3 Teach-pendent, execution "remote_control" file
# On HOST PC, in terminal
# roslaunch ur_robot_driver ur3_bringup.launch robot_ip:=192.168.0.9
# roslaunch ur3_moveit_config ur3_moveit_planning_execution.launch
# rosrun ~
