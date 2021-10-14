#!/usr/bin/env python
import time
import sys
from math import pi
import numpy as np

import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped, Vector3


from ur_kinematics.ur_kin_py import forward, inverse

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

def best_sol(sols, q_guess, weights):
    valid_sols = []
    for sol in sols:
        test_sol = np.ones(6)*9999.
        for i in range(6):
            for add_ang in [-2.*np.pi, 0, 2.*np.pi]:
                test_ang = sol[i] + add_ang
                if (abs(test_ang) <= 2.*np.pi and 
                    abs(test_ang - q_guess[i]) < abs(test_sol[i] - q_guess[i])):
                    test_sol[i] = test_ang
        if np.all(test_sol != 9999.):
            valid_sols.append(test_sol)
    if len(valid_sols) == 0:
        return None
    best_sol_ind = np.argmin(np.sum((weights*(valid_sols - np.array(q_guess)))**2,1))
    return valid_sols[best_sol_ind]

class LevelingIK:
    def __init__(self):
        #/vel_based_pos_traj_controller/
        self.client = actionlib.SimpleActionClient('manipulator_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.goal = FollowJointTrajectoryGoal()
        self.goal.trajectory = JointTrajectory()
        self.goal.trajectory.joint_names = JOINT_NAMES
        self.last_torque = None
        self.diff = 0
        self.move_dist = 0
        print "Waiting for server..."
        self.client.wait_for_server()
        print "Connected to server"
        joint_states = rospy.wait_for_message("joint_states", JointState)
        self.joints_pos_start = np.array(joint_states.position)
        self._wrench_sub = rospy.Subscriber('wrench', 
                                            WrenchStamped, 
                                            self._wrench_callback, 
                                            queue_size=10)
        rospy.Timer(rospy.Duration(1.), self.timer_cb)
        print "Init done"

    def move(self):
        diff_z = self.move_dist
        diff_z = 1.57 / 180. *10
        print diff_z
        if not diff_z==0.0:
            
            
            #self.joints_pos_start[3] = self.joints_pos_start[3]+diff_z
            #x = forward(self.joints_pos_start)
            #x[2, 3] = x[2, 3] + diff_z
            #sols = inverse(np.array(x), float(self.joints_pos_start[5]))
            #qsol = best_sol(sols, self.joints_pos_start, [1.]*6)
            qsol = True
            if qsol is not None:
                self.goal.trajectory.points = [
                    JointTrajectoryPoint(positions=self.joints_pos_start.tolist(), velocities=[0]*6, time_from_start=rospy.Duration(0.0)),

                    #JointTrajectoryPoint(positions=qsol.tolist(), velocities=[0]*6, time_from_start=rospy.Duration(1./125.))
                    JointTrajectoryPoint(positions=self.joints_pos_start.tolist()+[0,0,0,diff_z,0,0], velocities=[0]*6, time_from_start=rospy.Duration(1.))
                ]
                print 'start: ' + str(self.joints_pos_start.tolist())
                #print 'goal: ' + str(qsol.tolist())
                try:
                    self.client.send_goal(self.goal)
                    #self.joints_pos_start = qsol
                except:
                    raise

    def _wrench_callback(self, msg):
        running_avg = lambda x, y, a: x*a + (1.-a)*y
        if np.abs(msg.wrench.torque.y) > 0.1:
            #print msg.wrench.torque.y
            self.move_dist = running_avg(self.move_dist, msg.wrench.torque.y, 0.2)
        else:
            self.move_dist = running_avg(self.move_dist, 0., 0.2)
            #self.move_dist = running_avg(self.move_dist, 0.0, 0.2)
        
        # #print self.last_torque
        # if self.last_torque:
        #     self.diff = msg.wrench.torque.y - self.last_torque
        #     self.last_torque = msg.wrench.torque.y
        # else:
        #     self.last_torque = msg.wrench.torque.y
        #     self.diff = 0
        # if self.diff > 0.1:
        #     self.move_dist = running_avg(self.move_dist, 0.005, 0.2)
        # elif self.diff < -0.1:
        #     self.move_dist = running_avg(self.move_dist, -0.005, 0.2)
        # else:
        #     self.move_dist = running_avg(self.move_dist, 0.0, 0.2)

    def timer_cb(self, _event):
        print "Move dist: " + str(self.move_dist)
        self.move()


def main():
    rospy.init_node("test_move", anonymous=True, disable_signals=True)
    l_ik = LevelingIK()
    rospy.spin()

if __name__ == "__main__":
    main()