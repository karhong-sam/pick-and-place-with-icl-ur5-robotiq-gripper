#ifndef NAIVE_LEVELING_H
#define NAIVE_LEVELING_H

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <tf/tf.h>
#include <geometry_msgs/Pose.h>

#include <vector>
#include <boost/shared_ptr.hpp>

namespace ur5_control
{
//typedef boost::shared_ptr<moveit::planning_interface::MoveGroup> MoveGroupPtr;

class NaiveLeveling
{
public:
    NaiveLeveling(ros::NodeHandle &nh);
    virtual ~NaiveLeveling();
    bool planMove(const geometry_msgs::PoseConstPtr & target_pose);
    bool planMove(double x, double y, double z, double r, double p, double y);
    bool planMove(double x, double y, double z, double qw);

    // MoveGroupPtr getGroup() 
    // {
    //     return group_ptr_;
    // }
protected:
    //MoveGroupPtr group_ptr_;
    //robot_model::RobotModelPtr kinematic_model_;
    //robot_state::RobotStatePtr kinematic_state_;
    //robot_state::JointModelGroup * joint_model_group_; // bug somewhere
    //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::string group_name_;
    moveit::planning_interface::MoveGroupInterface move_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    moveit::planning_interface::MoveGroup::Plan my_plan_;
    bool success_;
    robot_state::RobotState start_state_;
    double tolerence_;


}; // class NaiveLeveling
} // namespace ur5_control


#endif