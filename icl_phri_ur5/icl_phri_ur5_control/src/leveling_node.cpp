#include <ros/ros.h>
#include "naive_leveling/naive_levelinng.h"

int main(int argc, char **argv)
{
    ros::init (argc, argv, "move_group_tutorial");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle;
}