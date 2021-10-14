#!/usr/bin/env python
'''
    DEPRECATED
'''
from __future__ import division, print_function
import numpy as np
from time import sleep


import rospy
from std_msgs.msg import String, Header
from geometry_msgs.msg import WrenchStamped, Vector3

from robotiq_c_model_control.msg import _CModel_robot_output as outputMsg
from robotiq_c_model_control.msg import _CModel_robot_input as inputMsg

from icl_phri_robotiq_control.robotiq_utils import *



def norm(vect):
    return (vect.x**2.0 + vect.y**2.0 + vect.z**2.0)**0.5

# class MyVector3:
#     def __init__(self, vect):
#         self.x = vect.x
#         self.y = vect.y
#         self.z = vect.z
#         self.mag = norm(vect)
#         self.dir = np.arrary([vect.x, vect.y, vect.z]) / self.mag



class ForceTorqueSensor:
    def __init__(self, topic_name):
        self.fts_sub = rospy.Subscriber(
            topic_name, WrenchStamped, self.fts_callback)
        self.header = Header()
        self.force = np.zeros(3)
        self.torque = np.zeros(3)

    def fts_callback(self, wrench_stamped):
        self.header = wrench_stamped.header
        self.force = vector3_to_numpy(wrench_stamped.wrench.force)
        self.torque = vector3_to_numpy(wrench_stamped.wrench.torque)

    def get_force(self):
        return self.force

    def get_torque(self):
        return self.torque

    # def get_force_mag(self):
    #     return self.force.mag

    # def get_force_dir(self):
    #     return self.force.dir


if __name__ == '__main__':
    rospy.init_node('robotiq_ctrl', anonymous=True)
    gripper_ctrl = RobotiqGripperCtrl()
    force_torque_sensor = ForceTorqueSensor('robotiq_force_torque_wrench')
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        print(force_torque_sensor.get_force())
        if force_torque_sensor.get_force()[2] < -25.0:
            gripper_ctrl.set_speed(150)
            gripper_ctrl.close_gripper()
            print ('Closing gripper')
        if force_torque_sensor.get_force()[2] > -20.0:
            gripper_ctrl.set_speed(150)
            print('curr_pos:' + str(gripper_ctrl.curr_pos))
            gripper_ctrl.close_gripper(
                0 if (gripper_ctrl.curr_pos < 5) else gripper_ctrl.curr_pos - 5)
            sleep(0.5)
            gripper_ctrl.close_gripper(
                0 if (gripper_ctrl.curr_pos < 10) else gripper_ctrl.curr_pos - 10)
            gripper_ctrl.open_gripper()
            print('Opening gripper')
        rate.sleep()
