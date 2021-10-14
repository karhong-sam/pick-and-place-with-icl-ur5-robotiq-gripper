#!/usr/bin/env python

from __future__ import division, print_function
import numpy as np

import rospy
import tf
from tf import TransformListener
from tf.transformations import *

class FTSCalibSampler:
    def __init__(self, filename='poses.txt'):
        print('init')
        self.tf = TransformListener()
        self.base_link = 'base_link'
        self.tool_link = 'fts_toolside'
        self.pose_cnt = 0
        self.file = open(filename, 'w+')
        try:
            self.tf.waitForTransform(self.base_link, self.tool_link, rospy.Time(), rospy.Duration(5.0))
        except tf.Exception: # likely a timeout
            print("Timeout while waiting for the TF transformation with the map!"
                         " Is someone publishing TF tansforms?\n ROS positions won't be available.")
            self.tf_running = False

    def update(self):
        if self.tf.frameExists(self.base_link) and self.tf.frameExists(self.tool_link):
            t = self.tf.getLatestCommonTime(self.base_link, self.tool_link)
            position, quaternion = self.tf.lookupTransform(self.base_link, self.tool_link, t)
            self.new_pose = position + list(euler_from_quaternion(quaternion))
            print(self.new_pose)

    def write(self):
        self.file.write('pose{}: {}\n'.format(self.pose_cnt, str(self.new_pose)))
        self.pose_cnt += 1

def main():
    rospy.init_node('fts_calib')
    fts = FTSCalibSampler()
    fts.update()
    while not rospy.is_shutdown():
        raw_input('press to continue')
        fts.update()
        fts.write()

if __name__ == '__main__': main()
