#!/usr/bin/env python

import numpy as np
from time import sleep
import copy

import rospy
import tf
import message_filters
from std_msgs.msg import String, Header
from geometry_msgs.msg import WrenchStamped, Vector3, PointStamped

from robotiq_c_model_control.msg import _CModel_robot_output as outputMsg
from robotiq_c_model_control.msg import _CModel_robot_input as inputMsg

from robotiq_ctrl import ForceTorqueSensor

from robotiq_utils import *

from tf_message_filter import TFMessageFilter


class MsgProc:
    '''
        For use in msg proc pipeline
    '''
    filter_cnt = 0
    _args_list = []
    _wrench = np.zeros(6)
    _trans_rot = np.zeros(6)

    def __init__(self):
        type(self).add_filter()
        self._idx = MsgProc.filter_cnt
        self.args_dict = type(self)._args_list[self._idx]

    @property
    def wrench(self):
        return type(self)._wrench

    @wrench.setter
    def wrench(self, input_data):
        type(self)._trans_rot = input_data

    @property
    def trans_rot(self):
        return type(self)._trans_rot

    @trans_rot.setter
    def trans_rot(self, input_data):
        type(self)._trans_rot = input_data

    @property
    def args_list(self):
        return type(self)._args_list

    @args_list.setter
    def args_list(self, args):
        type(self)._args_list = args
    
    @classmethod
    def add_filter(cls):
        cls.filter_cnt = cls.filter_cnt + 1


class AverageFilter(MsgProc):
    '''
        For use in msg proc pipeline
    '''
    def __init__(self):
        MsgProc.__init__(self)
        # self.window_size = MsgProc.args_list[self._idx].get('window_size', 120)
        window_size = rospy.get_param("~avg_filter/window_size", 10)
        self._queue = QueueSetLength(window_size)

    def _callback(self, data):
        self._queue.push(data)
        filtered = self._queue.average()
        return filtered

    @property
    def wrench(self):
        return MsgProc.wrench

    @wrench.setter
    def wrench(self, input_data):
        MsgProc.wrench = self._callback(input_data)

    @property
    def trans_rot(self):
        return MsgProc.trans_rot

    @trans_rot.setter
    def trans_rot(self, input_data):
        MsgProc.trans_rot = input_data


class GravityCompensation(MsgProc):
    '''
        For use in msg proc pipeline
    '''
    def __init__(self):
        MsgProc.__init__(self)
        
    def _callback(self, data):
        pass
        #return filtered

    @property
    def wrench(self):
        return MsgProc.wrench

    @wrench.setter
    def wrench(self, input_data):
        MsgProc.wrench = self._callback(input_data)

    @property
    def trans_rot(self):
        return MsgProc.trans_rot

    @trans_rot.setter
    def trans_rot(self, input_data):
        MsgProc.trans_rot = input_data


class FTSProcPipeline:
    def __init__(self, topic_name='robotiq_force_torque_wrench', filter_list=[], args_list=[]):
        rospy.init_node('message_processing', anonymous=True)
        MsgProc.args_list = args_list

        _fts_sub = message_filters.Subscriber(topic_name, WrenchStamped)
        _ts = TFMessageFilter(_fts_sub, 'base_link', 'fts_toolside', queue_size=100)
        _ts.registerCallback(self._fts_callback)
        self._processed_pub = rospy.Publisher('out', WrenchStamped, queue_size=1)

        self._filter_instances = []
        for f in filter_list:
            self._filter_instances.append(f())
        rospy.spin()

    def _fts_callback(self, wrench_stamped, trans_rot):
        header = wrench_stamped.header
        force = vector3_to_numpy(wrench_stamped.wrench.force)
        torque = vector3_to_numpy(wrench_stamped.wrench.torque)
        wrench_data = np.hstack( (force, torque) )

        MsgProc.wrench = wrench_data
        MsgProc.trans_rot = np.hstack(tuple(trans_rot))

        for i in self._filter_instances:
            i.wrench = MsgProc.wrench
            i.trans_rot = MsgProc.trans_rot
        
        self._processed_pub.publish(array_to_wrench_stamped(header, MsgProc.wrench))


if __name__ == '__main__':
    filter_list = [AverageFilter, GravityCompensation]
    
    MsgProcPipeline()
    