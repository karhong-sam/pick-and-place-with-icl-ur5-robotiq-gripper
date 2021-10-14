#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import tf2_ros
from math import pi

object_id = 'object_13'

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('grasp_object')

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

robot = moveit_commander.RobotCommander()

arm = moveit_commander.MoveGroupCommander("manipulator")
grp = moveit_commander.MoveGroupCommander("gripper")

#first move
rospy.loginfo('homing')
arm.set_named_target('home_pos2')
arm.go()
grp.set_named_target('open')
grp.go()

rospy.loginfo('homing done')

object_found = False

rate = rospy.Rate(1)

#this is work-around because sometimes tf is not available instantly
while not object_found:    
    try:
        rospy.loginfo('finding object position')
        object_pose = tfBuffer.lookup_transform('base_link', object_id, rospy.Time(0))
        #print(object_pose)
        rospy.loginfo('finding object done')
        object_found= True

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logwarn('failed to transform ')
        rate.sleep()
        continue

print(object_pose)
pose = arm.get_current_pose().pose
pose.position.x = object_pose.transform.translation.x + 0.045
pose.position.y = object_pose.transform.translation.y 
arm.set_pose_target(pose)
arm.go()

pose.position.z = object_pose.transform.translation.z + 0.15
arm.set_pose_target(pose)
arm.go()

grp.set_named_target('closed')
grp.go(wait=True)


pose.position.z += 0.2
arm.set_pose_target(pose)
arm.go()
arm.set_named_target('place_pose')
arm.go(wait=True)

grp.set_named_target('open')
grp.go()
arm.set_named_target('home_pos2')
arm.go()

# spin until ctrl-c
rospy.spin()
