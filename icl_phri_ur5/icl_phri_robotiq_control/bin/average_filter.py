#!/usr/bin/env python

import numpy as np
from time import sleep

import rospy
from std_msgs.msg import String, Header
from geometry_msgs.msg import WrenchStamped, Vector3

from robotiq_c_model_control.msg import _CModel_robot_output as outputMsg
from robotiq_c_model_control.msg import _CModel_robot_input as inputMsg

from robotiq_ctrl import ForceTorqueSensor

from icl_phri_robotiq_control.robotiq_utils import *


class AverageFilter:
    def __init__(self, window_size=10, topic_name='robotiq_force_torque_wrench'):
        
        window_size = rospy.get_param("~window_size", 10)
        rate = rospy.get_param('~rate', 125.0)
        self._msg = WrenchStamped()
        self._fts_sub = rospy.Subscriber(
            topic_name, WrenchStamped, self._fts_callback)
        self._filtered_pub = rospy.Publisher('out', WrenchStamped, queue_size=1)
        self._queue = QueueSetLength(window_size)
        rospy.Timer(rospy.Duration(1 / rate), self.timer_cb)
        

    def _fts_callback(self, wrench_stamped):
        header = wrench_stamped.header
        force = vector3_to_numpy(wrench_stamped.wrench.force)
        torque = vector3_to_numpy(wrench_stamped.wrench.torque)
        data = np.hstack( (force, torque ) )
        
        self._queue.push(data)
        filtered = self._queue.average()
        self._msg = array_to_wrench_stamped(header, filtered)
        # self._filtered_pub.publish(array_to_wrench_stamped(header, filtered))

    def timer_cb(self, _event):
        self._filtered_pub.publish(self._msg)


if __name__ == '__main__':
    rospy.init_node('average_filter', anonymous=True)
    try:
        AverageFilter(100)
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
    
