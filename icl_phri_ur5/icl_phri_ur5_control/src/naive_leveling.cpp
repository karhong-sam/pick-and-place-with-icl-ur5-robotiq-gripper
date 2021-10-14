#include <ros/ros.h>

#include "icl_phri_ur5_control/naive_leveling.h"

namespace ur5_control
{
NaiveLeveling::NaiveLeveling(ros::NodeHandle &nh):
    group_name_("manipilator"),
    move_group_(new moveit::planning_interface::MoveGroupInterface(group_name_)),
    start_state_(* move_group_.getCurrentState()),
    tolerence_(0.05)
{
    const robot_state::JointModelGroup * joint_model_group =
        start_state_.getJointModelGroup(group_name_);
    
    // orientation constraint on the ee
    ROS_DEBUG("Setting orientation constraint.");
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = "fts_toolside";
    ocm.header.frame_id = "base_link";
    ocm.orientation.w = 1.0;
    ocm.absolute_x_axis_tolerance = tolerence_;
    ocm.absolute_y_axis_tolerance = tolerence_;
    ocm.absolute_z_axis_tolerance = tolerence_;
    ocm.weight = 1.0;

    moveit_msgs::Constraints test_constraints;
    test_constraints.orientation_constraints.push_back(ocm);
    move_group_.setPathConstraints(test_constraints);

    geometry_msgs::Pose start_pose2;
    start_pose2.orientation.w = 1.0;
    start_pose2.position.x = 0.55;
    start_pose2.position.y = -0.05;
    start_pose2.position.z = 0.8;

    start_state_.setFromIK(joint_model_group, start_pose2);
    move_group_.setStartState(start_state); 
    ROS_DEBUG("Setting orientation constraint finished.");
}

bool NaiveLeveling::planMove(const geometry_msgs::Pose::ConstPtr &target_pose)
{
    move_group_.setPoseTarget(*target_pose);
    return (move_group_.plan(my_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("Leveling", "Plan (pose goal) %s", success ? "" : "FAILED");
}

bool NaiveLeveling::planMove(double x, double y, double z, double qw)
{
    geometry_msgs::Pose target_pose;
    target_pose.orientation.w = qw;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    move_group.setPoseTarget(target_pose);
    return (move_group_.plan(my_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("Leveling", "Plan (pose goal) %s", success ? "" : "FAILED");
}

bool NaiveLeveling::planMove(double x, double y, double z, double r, double p, double y)
{
    geometry_msgs::Pose target_pose;
    target_pose.orientation = tf::createQuaternionFromRPY(r, p, y);
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    move_group.setPoseTarget(target_pose);
    return (move_group_.plan(my_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("Leveling", "Plan (pose goal) %s", success ? "" : "FAILED");
}
} // namespace ur5_control