#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('grasp_object', anonymous=True)
robot = moveit_commander.RobotCommander()

arm = moveit_commander.MoveGroupCommander("manipulator")
grp = moveit_commander.MoveGroupCommander("gripper")
arm.set_named_target('home_pos2')
arm.go()

pose = arm.get_current_pose().pose #6dof: [x1,x2,x3,x4,x5,x6]
pose.position.x += 0.02 #abs position of end-effector
pose.position.y += 0.28  #abs position of end-effector
arm.set_pose_target(pose)
arm.go()

pose.position.z -= 0.15
arm.set_pose_target(pose)
arm.go()

grp.set_named_target('closed')
grp.go(wait=True)

pose.position.z += 0.2
arm.set_pose_target(pose)
arm.go()
arm.set_named_target('place_pose')
arm.go(wait=True)
pose = arm.get_current_pose().pose
pose.position.z -= 0.05
arm.set_pose_target(pose)
arm.go(wait=True)

grp.set_named_target('open')
grp.go()
arm.set_named_target('home_pos2')
arm.go()