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
Q_fetch = [1.5279078483581543, -2.149566952382223, -1.1292513052569788, 0.14056265354156494, 1.786577582359314, 1.541101336479187]
Q_wait = [0.6767016649246216, -1.2560237089740198, -1.9550021330462855, -0.019442383443013966, 1.5472047328948975, 1.5413050651550293]
Q_give = [-0.20308286348451787, -2.2909210363971155, -1.2152870337115687, 0.34171295166015625, 1.4318565130233765, 1.5419158935546875]
# Q1 = [1.6492750644683838, -1.8676283995257776, -1.7732299009906214, -1.0136101881610315, 1.5450127124786377, 1.6861138343811035] # above

# Q2 = [1.650378704071045, -1.964351002370016, -1.7765887419330042, -0.9513161818133753, 1.5492768287658691, 1.6860419511795044] # down

# Q3 = Q1

# # Q4 = [0.19515973329544067, -2.0777676741229456, -1.3407052198993128, -1.262717072163717, 1.5548584461212158, 0.08926524966955185] # above target
# # Q5 = [0.19016268849372864, -2.129251782094137, -1.3405488173114222, -1.2351320425616663, 1.5682377815246582, 0.09852081537246704] # on target

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

    def sphere_waypoint(self, direction, dist=0.5):
        #self.wpose = copy.deepcopy(self._group.get_current_pose().pose)
        n_dir = normalize(direction)
        self.wpose.position.x = n_dir[0] * np.sqrt(dist)
        self.wpose.position.y = n_dir[1] * np.sqrt(dist)
        self.wpose.position.z = n_dir[2] * np.sqrt(dist)
        self.wpose.orientation = tf.transformations.quaternion_from_euler(*direction).normalize()

    def move(self, qs):
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = JOINT_NAMES
        move_time = 2.0
        try:
            joint_states = rospy.wait_for_message("icl_phri_ur5/joint_states", JointState)
            joints_pos = joint_states.position
            #joints_pos = self._group.get_current_joint_values()
            g.trajectory.points = []
            time_from_start = 0.0
            g.trajectory.points.append(JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(time_from_start)))
            for q in qs:
                time_from_start = time_from_start + move_time
                g.trajectory.points.append(JointTrajectoryPoint(positions=q, velocities=[0]*6, time_from_start=rospy.Duration(time_from_start)))
            self.client.send_goal(g)
            return self.client.wait_for_result()
        except KeyboardInterrupt:
            self.client.cancel_goal()
            raise
        except:
            raise
        
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

class HandOver:
    def __init__(self):
        self._mg = MoveGroup()
        self._gripper_ac = RobotiqActionClient('icl_phri_gripper/gripper_controller')
        self._gripper_ac.wait_for_server()
        print('Init gripper')
        self._gripper_ac.initiate()

        self._myo_gest_sub = rospy.Subscriber('/myo_raw/myo_gest_str', 
                                              String, 
                                              self.myo_gest_callback, queue_size=10)
        self._myo_emg_sub = rospy.Subscriber('/myo_raw/myo_emg', 
                                              EmgArray, 
                                              self.myo_emg_callback, queue_size=10)
        self._wrench_sub = rospy.Subscriber('icl_phri_gripper/wrench_filtered', 
                                            WrenchStamped, 
                                            self._wrench_callback, 
                                            queue_size=11)
        self.emg_array = np.array([])
        self.cmd_flag = False
        self.euler = np.zeros(3)
        self.axes = 2
        self.effort = 0
        
        self.rest_counter = 0
        self.at_rest = False
        self.is_handing = False
        print(self._mg.move([Q_wait, Q_fetch]))
        self.wait_for_action = True

    def _wrench_callback(self, msg):
        if msg.wrench.force.z > 4 and self.wait_for_action:
            self._gripper_ac.send_goal(0.14)
            self.is_handing = False
            self.wait_for_action = False
            print('handed')
            print(self._mg.move([ Q_fetch]))
            self.wait_for_action = True

        elif msg.wrench.force.z < -5 and not self.is_handing and self.wait_for_action:
            self._gripper_ac.send_goal(0.0)
            self.is_handing = True
            print('fetched')
            print(self._mg.move([Q_wait]))
            self.wait_for_action = False


    def myo_gest_callback(self, msg):
        # if msg.data=='FIST' and not self.is_handing:
        #     print('prepare')
        #     self._mg.move([Q1, Q2])     
        #     self._gripper_ac.send_goal(0.0)
        #     print(self._mg.move([Q3]))
        #     self.is_handing = True
        return
        if msg.data=='FINGERS_SPREAD' and self.is_handing:
            print('handover')
            print(self._mg.move([Q_give]))
            self.wait_for_action = True
            #self._gripper_ac.send_goal(0.14)
            #self.is_handing = False
            # print('move away')
            # print(self._mg.move([Q1,Q2]))
    
    def myo_emg_callback(self, msg):
        self.emg_array = np.array(msg.data)
        if sum(filter(lambda x: x > 40, self.emg_array)) == 0 and self.is_handing and not self.wait_for_action:
            self.rest_counter += 1
            if self.rest_counter == 20:
                print('handover')
                print(self._mg.move([Q_give]))
                self.wait_for_action = True
        else: 
            self.rest_counter = 0
            self.at_rest = False
        
        

        
if __name__ == '__main__':
    rospy.init_node('naive_leveling', anonymous=True)
    h = HandOver()
    rospy.spin()
