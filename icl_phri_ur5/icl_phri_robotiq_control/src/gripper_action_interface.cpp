#include <ros/ros.h>
#include "icl_phri_robotiq_control/gripper_action_interface.h"

GripperActionClass::GripperActionClass()
{
    nh_priv_.param<std::string>("gripper_name", gripper_name_, "icl_phri_gripper/gripper_controller");
    nh_priv_.param<double>("position", position_, 0.14);
    ac_.reset(new actionlib::SimpleActionClient<control_msgs::GripperCommandAction>(gripper_name, true));
    ROS_INFO("Waiting for action server to start.");
    ac_.waitForServer();
    ROS_INFO("Action server started, sending goal.");
}

bool GripperActionClass::sendGoal(double position, double timeout=5.0)
{
    goal_.command.position = position;
    goal_.command.max_effort = 100.0;
    ac_.sendGoal(goal_);
    bool finished_before_timeout = ac_.waitForResult(ros::Duration(timeout));
    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac_.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");
    return finished_before_timeout;
}

bool GripperActionClass::closeGripper()
{
    return GripperActionClass::sendGoal(0.0);
}

bool GripperActionClass::openGripper()
{
    return GripperActionClass::sendGoal(0.14);
}