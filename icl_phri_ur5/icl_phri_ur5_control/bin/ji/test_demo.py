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
#from tf2_msgs import TFMessage
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

class MoveGroup:
    def __init__(self):
        print ("============ Starting Moveit setup==============")
        moveit_commander.roscpp_initialize(sys.argv)
        #rospy.init_node('move_group_python_interface',
                  #anonymous=True)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander("manipulator")
        display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory,
                                      queue_size=20)
        
        print ("============ Reference frame: %s" % self._group.get_planning_frame())
        print ("============ End effector: %s" % self._group.get_end_effector_link())
        print ("============ Robot Groups:")
        print (self._robot.get_group_names())
        print ("============ Printing robot state")
        print (self._robot.get_current_state())
        print ("============")
        self.waypoints = []
        self.waypoints.append(self._group.get_current_pose().pose) # start with the current pose
        self.wpose = geometry_msgs.msg.Pose()
        
    def append_waypoint(self, Q):
        self.wpose = copy.deepcopy(self._group.get_current_pose().pose)
        #self.wpose.position.z = self._group.get_current_pose().pose.position.z + diff
        self.wpose.position.x = Q[0]
        self.wpose.position.y = Q[1]
        self.wpose.position.z = Q[2]
        self.wpose.orientation.x = Q[3]
        self.wpose.orientation.y = Q[4]
        self.wpose.orientation.z = Q[5]
        self.wpose.orientation.w = Q[6]
        self.waypoints.append(copy.deepcopy(self.wpose))
        print('waypount', self.waypoints)
        (cartesian_plan, fraction) = self._group.compute_cartesian_path(
                                     self.waypoints,   # waypoints to follow
                                     0.005,        # eef_step, 1cm
                                     0.0)         # jump_threshold, disabling
        rospy.sleep(1)
        self._group.execute(cartesian_plan)
        self.waypoints.pop(0)


    def move(self, Qs):
        for Q in Qs:
            self.append_waypoint(Q)
        
        
    def get_pose(self):
        return self._group.get_current_pose().pose

    def shift_pose_target(self, axis, value):
        axis_dict = {'x': 0, 'y': 1, 'z': 2, 'r': 3, 'p': 4, 'y': 5}
        print('ee: %s' % self._group.get_end_effector_link())
        self._group.shift_pose_target(axis_dict[axis], value, self._group.get_end_effector_link())
        #self.pose_target = self.get_pose()

    # def plan_move(self):
    #     print("============ Generating plan")
    #     self.plan = self._group.plan()

    # def go(self):
    #     self._group.go(wait=True)

    # def stop(self):
    #     self._group.stop()

    # def __del__(self):
    #     moveit_commander.roscpp_shutdown()

class HandOver:
    def __init__(self):
        self._mg = MoveGroup()
        self._gripper_ac = RobotiqActionClient('icl_phri_gripper/gripper_controller')
        self._gripper_ac.wait_for_server()
        print('============Init gripper============')
        self._gripper_ac.initiate()
        self.listener = tf.TransformListener()
        self.fetch = False

#-----------------get two frame transformation------------------#
    def get_transform(self, frame1, frame2):
        rate = rospy.Rate(10.0)
        rate.sleep()        
        try:
            (trans, rot) = self.listener.lookupTransform(frame1, frame2, rospy.Time(0))
            return trans, rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("wait for tf")

   
    def _list_add(self, ls1, ls2):
        return [x + y for x, y in zip(ls1, ls2)]


    def get_handover_pos(self):
        (trans1, rot1) = self.get_transform('/left_hand_1', '/base_link')
        (trans2, rot2) = ([0, 0, 0.1],
                            [0, 0, -0.7, 0.6])
        #rot_new = tf.transformations.quaternion_multiply(rot2, rot1)
        #tf.transformations.quaternion_from_euler()
        trans_new = self._list_add(trans1, trans2)
        # Q = trans_new + list(rot_new)
        Q = trans_new + list(rot2)
        return Q
        

    def frame_action(self, trans):
        if abs(trans[0]) > 0.6 and not self.fetch:
            rospy.sleep(5)
            Q_new = self.get_handover_pos()
            print('Q_new', Q_new)
            print(self._mg.move([Q_new]))
            self.fetch = True
        # if abs(trans[0]) < 0.4 and self.fetch:
        #     print(self._mg.move([Q_wait]))
        #     self.fetch = False
        
if __name__ == '__main__':
    rospy.init_node('test', anonymous=True)
    h = HandOver()
    print(rospy.is_shutdown)
    while not rospy.is_shutdown():
        try:
            rate = rospy.Rate(10.0)
            rate.sleep()
            trans = h.get_transform('/left_hand_1', '/right_hand_1')[0]
            # print("hand_gripper_tf_trans:{}\nhand_gripper_tf_quat:{}".format(
            #     h.get_transform('/ee_link', '/right_hand_1')[0],
            #     h.get_transform('/ee_link', '/right_hand_1')[1]
            # ))
            h.frame_action(trans)
        except:
            print('no action')


