#!/usr/bin/env python

from __future__ import division, print_function
import numpy as np
from time import sleep
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation

import rospy
from std_msgs.msg import String, Header
from geometry_msgs.msg import WrenchStamped, Vector3
import tf

from icl_phri_robotiq_control.robotiq_utils import *
#from utils import *

norm = lambda a:(a.x+a.y+a.z)/3.
to_quat = lambda o: np.array([o.x, o.y, o.z, o.w])
rad_to_ang = lambda x: x / np.pi * 180.
ang_to_rad = lambda x: x / 180. * np.pi
class MoveGroup:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander("manipulator")

        ## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
        print("============ Waiting for RVIZ...")
        #group_variable_values = self._group.get_current_joint_values()
        #initial_position = [0.0, -1.2, 1.8, -0.6, 1.6, 0.0]
        #group_variable_values = list(initial_position)
        #self._group.set_joint_value_target(group_variable_values) #Set a joint target
        #initial_plan=self._group.plan()
        #self._group.execute(initial_plan)
        #rospy.sleep(2)  #Wait for RViz to visualize
        print("============ Starting ")
        self.waypoints = []
        self.waypoints.append(self._group.get_current_pose().pose) # start with the current pose
        #self.base_xyz = [self.waypoints[0].position.x, self.waypoints[0].position.y, self.waypoints[0].position.z ] #Initial position is the basement
        
        self.wpose = geometry_msgs.msg.Pose()
        self.wpose.orientation.w = 1.0 # first orient gripper and move forward (+x)

    def append_waypoint(self, quat):
	    self.wpose = copy.deepcopy(self._group.get_current_pose().pose)
        #self.wpose.position.z = self._group.get_current_pose().pose.position.z + diff
        self.wpose.orientation.x = quat[0]
        self.wpose.orientation.y = quat[1]
        self.wpose.orientation.z = quat[2]
        self.wpose.orientation.w = quat[3]
        self.waypoints.append(copy.deepcopy(self.wpose))
        (cartesian_plan, fraction) = self._group.compute_cartesian_path(
                                     self.waypoints,   # waypoints to follow
                                     0.005,        # eef_step, 1cm
                                     0.0)         # jump_threshold, disabling
        self._group.execute(cartesian_plan)
        self.waypoints.pop(0)

    def append_angle_waypoint(self, ang):
        self.wpose = copy.deepcopy(self._group.get_current_pose().pose)
        quat_incre = tf.transformations.quaternion_from_euler(*list(ang))
        quat_orig = to_quat(self.wpose.orientation)
        quat_goal = tf.transformations.quaternion_multiply(quat_orig, quat_incre)
        print("orig:{}, incre:{}, new:{}".format(quat_orig, quat_incre, quat_goal))
        self.wpose.orientation.x = quat_goal[0]
        self.wpose.orientation.y = quat_goal[1]
        self.wpose.orientation.z = quat_goal[2]
        self.wpose.orientation.w = quat_goal[3]
        self.waypoints.append(copy.deepcopy(self.wpose))
        (cartesian_plan, fraction) = self._group.compute_cartesian_path(
                                     self.waypoints,   # waypoints to follow
                                     0.01,        # eef_step, 1cm
                                     0.0)         # jump_threshold, disabling
        self._group.execute(cartesian_plan)#, wait=False)
        self.waypoints.pop(0)

        

    #def update_pose(self, diff):
        

    def get_pose(self):
        return self._group.get_current_pose().pose

    def shift_pose_target(self, axis, value):
        axis_dict = {'x': 0, 'y': 1, 'z': 2, 'r': 3, 'p': 4, 'y': 5}
        print('ee: %s' % self._group.get_end_effector_link())
        self._group.shift_pose_target(axis_dict[axis], value, self._group.get_end_effector_link())
        #self.pose_target = self.get_pose()

    def plan_move(self):
        print("============ Generating plan")
        self.plan = self._group.plan()

    def go(self):
        self._group.go(wait=True)

    def stop(self):
        self._group.stop()

    def __del__(self):
        moveit_commander.roscpp_shutdown()


class NaiveLeveling:
    def __init__(self):
        # r = rospy.Rate(15)
        self._mg = MoveGroup()
        self._gripper_ac = RobotiqActionClient('icl_phri_gripper/gripper_controller')
        #self._myo_sub = rospy.Subscriber('/myo_raw/pose', 
        #                                    geometry_msgs.msg.PoseStamped, 
        #                                    self._myo_callback, 
        #                                    queue_size=1)
        self._myo_gest_sub = rospy.Subscriber('/myo_raw/myo_gest_str', 
                                            String, 
                                            self._myo_gest_callback, queue_size=1)
        self.cmd_flag = False
        self.euler = np.zeros(3)
        self.axes = 2
	self.effort = 0
        #print(self._mg.get_pose())
        #self.last_torque = None
        #self.diff = 0
        # self._mg.update_pose_target()
        #r.sleep()
        rospy.Timer(rospy.Duration(0.01), self.timer_cb)
        

    def _myo_gest_callback(self, msg):
        if msg.data=='FIST':
             print(self._gripper_ac.send_goal(0.0))
        if msg.data=='FINGERS_SPREAD':
             print(self._gripper_ac.send_goal(0.14))
        if msg.data=='WAVE_IN':
             self.cmd_flag = True
             self.euler = np.zeros(3)
             self.euler[self.axes] = ang_to_rad(5.0)
             #print(output)
             #self._mg.append_angle_waypoint(output)
        if msg.data=='WAVE_OUT':
             self.cmd_flag = True
             self.euler = np.zeros(3)
             self.euler[self.axes] = ang_to_rad(-5.0)
             #self._mg.append_angle_waypoint(output)
        if msg.data=='REST':
             #self._mg.stop()
             self.euler = np.zeros(3)
             self.cmd_flag = False
        
        if msg.data=='THUMB_TO_PINKY': 
             #self.axes = (self.axes + 1) % 3
             print(self.axes)

    def _myo_callback(self, msg):
	
	#self.effort = 0.2 * self.effort + 0.8 * msg.wrench.torque.y
	#print(msg.pose.orientation)
        quat_orig = to_quat(self._mg.get_pose().orientation)
        quat_incre = to_quat(msg.pose.orientation)
        quat_incre = np.around(quat_incre, decimals=1)
        print(quat_incre)
        if np.max(np.abs(quat_incre[0:3])) <= 0.5:
            quat_goal = tf.transformations.quaternion_multiply(quat_orig, quat_incre)
            self._mg.append_waypoint(quat_goal)

    def timer_cb(self, _event):
       if self.cmd_flag:
           self._mg.append_angle_waypoint(self.euler)
        

        
if __name__ == '__main__':
    rospy.init_node('naive_leveling', anonymous=True)
    NaiveLeveling()
    rospy.spin()
