#!/usr/bin/env python

import sys
import copy
import rospy
from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation
from trajectory_msgs.msg import JointTrajectoryPoint

if __name__=='__main__':

    roscpp_initialize(sys.argv)
    rospy.init_node('moveit_py_demo', anonymous=True)
    
    scene = PlanningSceneInterface()
    robot = RobotCommander()
    arm = MoveGroupCommander("manipulator")
    gripper = MoveGroupCommander("end_effector")
    rospy.sleep(1)

    # clean the scene
    scene.remove_world_object("table")
    scene.remove_world_object("part")

    arm.set_named_target("up")
    arm.go()
    
    gripper.set_named_target("open")
    gripper.go()
    
    rospy.sleep(1)

    # publish a demo scene
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()

    # add a table
    p.pose.position.x = 0.42
    p.pose.position.y = -0.2
    p.pose.position.z = 0.3
    scene.add_box("table", p, (0.5, 1.5, 0.6))

    # add an object to be grasped
    p.pose.position.x = 0.205
    p.pose.position.y = -0.12
    p.pose.position.z = 0.7
    scene.add_box("part", p, (0.07, 0.01, 0.2))
    
    rospy.sleep(1)
    
    grasps = []
    
    g = Grasp()
    g.id = "test"
    grasp_pose = PoseStamped()
    grasp_pose.header.frame_id = "world"
    grasp_pose.pose.position.x = 0.148554
    grasp_pose.pose.position.y = -0.116075
    grasp_pose.pose.position.z = 0.70493
    grasp_pose.pose.orientation.x = -0.709103
    grasp_pose.pose.orientation.y = 0.0137777
    grasp_pose.pose.orientation.z = 0.0164031
    grasp_pose.pose.orientation.w = 0.704779
    
    arm.set_pose_target(grasp_pose)
    arm.go()
    
    rospy.sleep(2)
    
    # set the grasp pose
    g.grasp_pose = grasp_pose
    
    # define the pre-grasp approach
    g.pre_grasp_approach.direction.header.frame_id = "world"
    g.pre_grasp_approach.direction.vector.x = 1.0
    g.pre_grasp_approach.direction.vector.y = 0.0
    g.pre_grasp_approach.direction.vector.z = 0.0
    g.pre_grasp_approach.min_distance = 0.001
    g.pre_grasp_approach.desired_distance = 0.1
    
    g.pre_grasp_posture.header.frame_id = "tool0"
    g.pre_grasp_posture.joint_names = ["finger_joint"]
    
    pos = JointTrajectoryPoint()
    pos.positions.append(0.0)
    
    g.pre_grasp_posture.points.append(pos)
    
    # set the grasp posture
    g.grasp_posture.header.frame_id = "tool0"
    g.grasp_posture.joint_names = ["finger_joint"]

    pos = JointTrajectoryPoint()
    pos.positions.append(0.2)
    pos.effort.append(0.0)
    
    g.grasp_posture.points.append(pos)

    # set the post-grasp retreat
    g.post_grasp_retreat.direction.header.frame_id = "world"
    g.post_grasp_retreat.direction.vector.x = 0.0
    g.post_grasp_retreat.direction.vector.y = 0.0
    g.post_grasp_retreat.direction.vector.z = 1.0
    g.post_grasp_retreat.desired_distance = 0.25
    g.post_grasp_retreat.min_distance = 0.01

    g.allowed_touch_objects = ["table"]

    g.max_contact_force = 0
    
    # append the grasp to the list of grasps
    grasps.append(g)

    rospy.sleep(2)

    # pick the object
    robot.arm.pick("part", grasps)

    rospy.spin()
    roscpp_shutdown()
