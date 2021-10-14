#!/usr/bin/env python

from __future__ import division, print_function
import numpy as np
from math import *
from time import sleep
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState

import rospy
from ros_myo.msg import EmgArray
from std_msgs.msg import String, Header
from geometry_msgs.msg import WrenchStamped, Vector3
import tf
from tf.transformations import *

from icl_phri_robotiq_control.robotiq_utils import *

normalize = lambda x: x/np.sqrt(x[0]**2.+x[1]**2.+x[2]**2.)
norm = lambda a:np.sqrt(a.x**2.+a.y**2.+a.z**2.)
to_quat = lambda o: np.array([o.x, o.y, o.z, o.w])
rad_to_ang = lambda x: x / np.pi * 180.
ang_to_rad = lambda x: x / 180. * np.pi
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
class MoveGroup:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander("manipulator")
        self.waypoints = []
        self.waypoints.append(self._group.get_current_pose().pose) # start with the current pose
        #self.base_xyz = [self.waypoints[0].position.x, self.waypoints[0].position.y, self.waypoints[0].position.z ]
        
        self.wpose = geometry_msgs.msg.Pose()
        self.wpose.orientation.w = 1.0 # first orient gripper and move forward (+x)
        self.client = actionlib.SimpleActionClient('icl_phri_ur5/follow_joint_trajectory', FollowJointTrajectoryAction)
        print ("Waiting for server...")
        self.client.wait_for_server()
        
    def move(self, q, move_time):
        q = ang_to_rad(q)
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = JOINT_NAMES
        try:
            joint_states = rospy.wait_for_message("icl_phri_ur5/joint_states", JointState)
            joints_pos = joint_states.position
            new_joints_pos = list(copy.deepcopy(joints_pos))
            new_joints_pos[5] = new_joints_pos[5] + q
            #joints_pos = self._group.get_current_joint_values()
            g.trajectory.points = []
            time_from_start = 0.0
            g.trajectory.points.append(JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(time_from_start)))
            time_from_start = time_from_start + move_time
            g.trajectory.points.append(JointTrajectoryPoint(positions=new_joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(time_from_start)))
            self.client.send_goal(g)
            return self.client.wait_for_result()
        except KeyboardInterrupt:
            self.client.cancel_goal()
            raise
        except:
            raise

    def __del__(self):
        moveit_commander.roscpp_shutdown()

class HandOver:
    def __init__(self):
        self._mg = MoveGroup()
        self._gripper_ac = RobotiqActionClient('icl_phri_gripper/gripper_controller')
        self._gripper_ac.wait_for_server()
        print('Init gripper')
        self._gripper_ac.initiate()
        self._wrench_sub = rospy.Subscriber('icl_phri_gripper/robotiq_force_torque_wrench', 
                                            WrenchStamped, 
                                            self._wrench_callback, 
                                            queue_size=1)
        self.torque_z = 0

    def _wrench_callback(self, msg):
        self.torque_z = msg.wrench.torque.z + 0.142
        print (self.torque_z)
        if self.torque_z > 0.3: # ccw
            self._mg.move(30, 1)
        elif self.torque_z < -0.3: # cw 
            self._mg.move(-30, 1)

if __name__ == '__main__':
    rospy.init_node('naive_leveling', anonymous=True)
    h = HandOver()
    rospy.spin()
