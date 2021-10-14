from __future__ import print_function

import numpy as np
import sys
import copy
from time import sleep

import rospy
import moveit_commander
import actionlib

from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from geometry_msgs.msg import WrenchStamped, Vector3, Wrench
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String
import moveit_msgs.msg
import geometry_msgs.msg

from robotiq_c_model_control.msg import _CModel_robot_output as outputMsg
from robotiq_c_model_control.msg import _CModel_robot_input as inputMsg


def vector3_to_numpy(vect):
    return np.array([vect.x, vect.y, vect.z])

def array_to_wrench(array):
    msg = Wrench()
    msg.force.x = array[0]
    msg.force.y = array[1]
    msg.force.z = array[2]
    msg.torque.x = array[3]
    msg.torque.y = array[4]
    msg.torque.z = array[5]
    return msg

def array_to_wrench_stamped(header, array):
    msg = WrenchStamped()
    msg.header = header
    msg.wrench = array_to_wrench(array)
    return msg

class RobotiqGripperCtrl:
    def __init__(self, gripper_name):
        # input gACT gGTO gSTA gOBJ gFLT gPR gPO gCU
        self.query = inputMsg.CModel_robot_input()
        # self.activated = False
        self.current = 0.0
        self.requested_pos = 0
        self.curr_pos = 0
        # gOBJ: Only valid if gGTO = 1.
        # 0x00 - Fingers are in motion towards requested position. No object detected.
        # 0x01 - Fingers have stopped due to a contact while opening before requested position. Object detected.
        # 0x02 - Fingers have stopped due to a contact while closing before requested position. Object detected.
        # 0x03 - Fingers are at requested position. No object detected or object has been lost / dropped.
        self.grasp_status = 0
        # output_msg rACT rGTO rATR rPR rSP rFR
        self.command = outputMsg.CModel_robot_output()
        self.command_pub = rospy.Publisher(
            gripper_name + '/output', outputMsg.CModel_robot_output, queue_size=1)
        print("init gripper")
        self.reset()
        sleep(1)
        self.activate()
        sleep(1)
        print("finished")
        self.query_sub = rospy.Subscriber(
            gripper_name + '/input', inputMsg.CModel_robot_input, self.query_callback)

    def query_callback(self, input):
        #print("callback")
        #print(self.query.gSTA)
        self.query = input
        self.activated = True if self.query.gSTA == 1 else False

        if self.query.gSTA == 1:                          # activaton in progress
            print("Activation in progress")
        elif not self.query.gSTA == 3:                    # not activated
            print("Not activated. Try to activate.")
            self.activate()
        else:                                           # activated
            #self.activated = True
            self.grasp_status = self.query.gOBJ
            self.requested_pos = self.query.gPR
            self.curr_pos = self.query.gPO

    def reset(self):
        print('reset')
        self.command = outputMsg.CModel_robot_output()
        self.command.rACT = 0
        self.command_pub.publish(self.command)

    def activate(self):
        # if not self.activated:
        print('activate')
        self.command = outputMsg.CModel_robot_output()
        self.command.rACT = 1
        self.command.rGTO = 1
        self.command.rSP = 255
        self.command.rFR = 150
        self.command_pub.publish(self.command)
        self.activated = True

    def close_gripper(self, position=255):
        self.command.rPR = position
        self.send_command()

    def open_gripper(self):
        self.command.rPR = 0
        self.send_command()

    def set_speed(self, speed=150):
        self.command.rSP = speed
        # self.command_pub.publish(command)

    def set_force(self, force=127):
        self.command.rFR = force
        # self.command_pub.publish(command)

    def send_command(self):
        if self.command.rACT == 1:
            self.command_pub.publish(self.command)
        else:
            print("Not Activated.")

    def is_grasp_success(self):
        print("Grasp status: " + str(self.grasp_status))
        if self.grasp_status == 2:
            return True
        else:
            return False


class RobotiqActionClient:
    '''
        Action client to control robotiq gripper
    '''
    def __init__(self, gripper_name):
        self._gripper_name = gripper_name
        self._ac = actionlib.SimpleActionClient(self._gripper_name, GripperCommandAction)
        self._goal = GripperCommandGoal()

        # Wait 10 Seconds for the gripper action server to start or exit
        if not self._ac.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("Exiting - %s Gripper Action Server Not Found" %
                         (self._gripper_name.capitalize(),))
            rospy.signal_shutdown("Action Server not found")
            sys.exit(1)
        self.clear()

    def initiate(self):
        self._gripper_ctrl = RobotiqGripperCtrl(self._gripper_name)

    def send_goal(self, position, effort=100.0, timeout=5.0):
        self._goal.command.position = position
        self._goal.command.max_effort = effort
        self._ac.send_goal(self._goal)
        self._ac.wait_for_result(timeout=rospy.Duration(timeout))
        return self._ac.get_result()
    
    def wait_for_result(self):
        return self._ac.wait_for_result()

    def wait_for_server(self):
        return self._ac.wait_for_server()

    def cancel(self):
        self._ac.cancel_goal()

    def clear(self):
        self._goal = GripperCommandGoal()


class QueueSetLength:
    '''A container with a first-in-first-out (FIFO) queuing policy with a set length.'''

    def __init__(self, length):
        self._list = np.zeros(6)
        self._length = length

    def push(self, data):
        '''Enqueue the 'item' into the queue'''
        # print(self._list)
        self._list = np.vstack( ( data, self._list ) )
        if self._list.shape[0] > self._length:
            self.pop()
        
        return self._list

    def pop(self):
        '''
        Dequeue the earliest enqueued item still in the queue. This
        operation removes the item from the queue.
        '''
        
        self._list = self._list[:-1]
        return self._list

    def isEmpty(self):
        '''Returns true if the queue is empty'''
        return self._list.size == 0

    def average(self):
        return np.mean(self._list, axis=0)


