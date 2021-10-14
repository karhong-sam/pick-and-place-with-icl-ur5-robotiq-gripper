## Dependance
[universal_robot](https://github.com/ros-industrial/universal_robot)

[ur_modern_driver](https://github.com/ros-industrial/ur_modern_driver)

[robotiq](https://github.com/ros-industrial/robotiq)

[ros_canopen](https://github.com/ros-industrial/ros_canopen)

## How to use:
### Find the robot ip address
UR’s teach-pendant -> Setup Robot -> Setup Network Menu -> ip address
### Bring up the robot:
`roslaunch icl_phri_ur5_gripper_bringup ur5_bringup.launch with_gripper:=true limited:=true robot_ip:=<robot ip address>`

### Launch Moveit! scence:
`roslaunch icl_phri_ur5_gripper_moveit_config ur5_gripper_moveit_planning_execution.launch limited:=true`

`roslaunch icl_phri_ur5_gripper_moveit_config moveit_rviz.launch config:=true`

## If you want to bring up ur5 and Moveit! at once:
`roslaunch icl_phri_ur5_gripper_bringup ur5_moveit_bringup.launch`

## Control the Gripper:
use the ur5_gripper_control package [here](https://github.com/intuitivecomputing/PATI)
