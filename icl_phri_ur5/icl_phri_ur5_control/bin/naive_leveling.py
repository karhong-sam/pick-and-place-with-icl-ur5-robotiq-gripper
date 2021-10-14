#!/usr/bin/env python

from __future__ import print_function
import numpy as np
from time import sleep
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation

import rospy
from std_msgs.msg import String, Header
from geometry_msgs.msg import WrenchStamped, Vector3

#from utils import *

norm = lambda a:(a.x+a.y+a.z)/3.

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
        rospy.sleep(2)  #Wait for RViz to visualize
        print("============ Starting ")
        self.waypoints = []
        self.waypoints.append(self._group.get_current_pose().pose) # start with the current pose
        #self.base_xyz = [self.waypoints[0].position.x, self.waypoints[0].position.y, self.waypoints[0].position.z ] #Initial position is the basement
        self.wpose = geometry_msgs.msg.Pose()
        self.wpose.orientation.w = 1.0 # first orient gripper and move forward (+x)

    def append_waypoint(self, diff):
	    self.wpose = copy.deepcopy(self._group.get_current_pose().pose)
        self.wpose.position.z = self._group.get_current_pose().pose.position.z + diff
        self.waypoints.append(copy.deepcopy(self.wpose))
        (cartesian_plan, fraction) = self._group.compute_cartesian_path(
                                     self.waypoints,   # waypoints to follow
                                     0.005,        # eef_step, 1cm
                                     0.0)         # jump_threshold, disabling
        self._group.execute(cartesian_plan)
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

    def __del__(self):
        moveit_commander.roscpp_shutdown()


class NaiveLeveling:
    def __init__(self):
        rospy.init_node('naive_leveling', anonymous=True)
        # r = rospy.Rate(15)
        self._mg = MoveGroup()
        self._wrench_sub = rospy.Subscriber('icl_phri_gripper/wrench_filtered', 
                                            WrenchStamped, 
                                            self._wrench_callback, 
                                            queue_size=1)
	    self.effort = 0
        #print(self._mg.get_pose())
        #self.last_torque = None
        #self.diff = 0
        # self._mg.update_pose_target()
        #r.sleep()
        rospy.spin()

    def _wrench_callback(self, msg):
	
	#self.effort = 0.2 * self.effort + 0.8 * msg.wrench.torque.y
	print((msg.wrench.torque.y-0.3))
        if msg.wrench.torque.y > 0.6 or msg.wrench.torque.y < 0:
            self._mg.append_waypoint(-(msg.wrench.torque.y-0.3)*0.01)
        '''
        if self.last_torque:
            self.diff = msg.wrench.torque.y - self.last_torque
            self.last_torque = msg.wrench.torque.y
        else:
            self.last_torque = msg.wrench.torque.y
            self.diff = 0
        if self.diff > 0.1:
            self._mg.shift_pose_target('z', 0.005)
            self._mg.plan_move()
            self._mg.go()
        elif self.diff < -0.1:
            self._mg.shift_pose_target('z', -0.005)
            self._mg.plan_move()
            self._mg.go()
        print(self.diff)  
        print(self._mg.pose_target.position) 
        '''

        
if __name__ == '__main__':
    NaiveLeveling()
