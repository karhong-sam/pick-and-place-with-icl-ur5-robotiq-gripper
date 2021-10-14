#!/usr/bin/env python
from __future__ import division, print_function
import sys
import numpy as np
from time import sleep
import argparse

import rospy


from icl_phri_robotiq_control.robotiq_utils import *


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--position', '-p', metavar='N', type=float, help='Position in meter.0.0-0.14')
    parser.add_argument('--effort', '-e', metavar='N', type=float, help='effort 0-100')
    parser.add_argument('--init', '-i', metavar='0/1', type=int, nargs='?', help='initialization', const=1, default=0)
    parser.add_argument('--ns', '-n', type=str, nargs='?', help='namespace', const="", default="icl_phri_gripper")
    args = parser.parse_args()
    
    rospy.init_node('gripper_action_client')
    print ("ns" + str(args.ns))
    gripper_name = rospy.get_param('~gripper_name', args.ns + '/gripper_controller')
    #gripper_name = rospy.get_param('~gripper_name', 'gripper_controller')
    gripper_client = RobotiqActionClient(gripper_name)
    if args.init: 
        #RobotiqGripperCtrl(gripper_name)
        gripper_client.initiate()
        print(gripper_client.send_goal(0.14))
        
    if args.position is not None:
        if args.effort is not None:
            print(gripper_client.send_goal(args.position, effort=args.effort))
        else:
            print(gripper_client.send_goal(args.position))
        

if __name__ == '__main__':
    main()
