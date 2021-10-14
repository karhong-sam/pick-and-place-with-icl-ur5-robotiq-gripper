#ifndef GRIPPER_ACTION_INTERFACE_H
#define GRIPPER_ACTION_INTERFACE_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/GripperCommandAction.h>


class GripperActionClass
{
public:
    GripperActionClass();
    bool sendGoal(double position, , double timeout); // in meter
    bool closeGripper();
    bool openGripper();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_("~");
    std::string gripper_name_;
    double position_;
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> ac_;
    control_msgs::GripperCommandGoal goal_;
};

#endif