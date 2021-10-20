# pick-and-place-with-icl-ur5-robotiq-gripper

## Prerequisite:

1. Ubuntu 18.04 LTS
2. [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
3. [MoveIt!](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/getting_started/getting_started.html)
4. [OpenNI Kinect Package (Camera)](https://www.oreilly.com/library/view/learning-robotics-using/9781788623315/1235f7fe-3637-412a-a386-05859b89ee67.xhtml)


## Getting Started:

### 1. Create new catkin workspace:

    mkdir -p ur5_robotiq_ws/src
    
### 2. Git clone to the ur5_robotiq_ws/src:

    cd ur5_robotiq_ws/src

    git clone https://github.com/khs-sm/pick-and-place-with-icl-ur5-robotiq-gripper.git
    
### 3. Make sure the packages is in the directory as below:
```
ur5_robotiq_ws   
│
└───src
    │   find-object
    │   gazebo-pkgs
    |   general-message-pkgs
    |   ......
    |   README.md

```
### 4. Install dependencies using rosdep install and perform catkin_make to build the project:

   `cd ur5_robotiq_ws` or `cd ..`
   
   `rosdep install --from-paths src --ignore-src -r -y`
   
   `catkin_make`
   
   source the file before launch
   
   `source devel/setup.bash`
   
## Gazebo Simulation

### 1. Launch the UR5 Robotiq Gripper:

    roslaunch icl_ur5_setup_gazebo icl_ur5_gripper.launch
    
### 2. Enable ROS Control on Simulation from MoveIt:

    roslaunch icl_ur5_setup_moveit_config ur5_gripper_moveit_planning_execution.launch sim:=true
   
### 3. Launch Rviz with MoveIt:

    roslaunch icl_ur5_setup_moveit_config moveit_rviz.launch config:=true
    
### 4. Test the Simulation:

    rosrun scripts test_grasp.py
    
![pick_place_test](https://user-images.githubusercontent.com/59763695/138029853-1d45df31-9ce7-4690-9e03-8bfdcff6afa1.gif)
    
### (OPTIONAL) Launch Object Detection with find_object_2d:

    roslaunch find-object start_find_object_3d_session.launch
    
### (OPTIONAL) Apply Object Detection to Motion Planning:

    rosrun scripts vision_grasp.py
    
## Debug

check on [here](https://github.com/khs-sm/pick-and-place-with-icl-ur5-robotiq-gripper/wiki)
    
## References
1. [universal robot](https://github.com/ros-industrial/universal_robot)
2. [robotiq](https://github.com/ros-industrial/robotiq)
3. [icl_phri_ur5](https://github.com/intuitivecomputing/icl_phri_ur5)
4. [find_object_2d](https://github.com/introlab/find-object)

