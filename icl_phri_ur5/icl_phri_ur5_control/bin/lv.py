import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import leaper

from numpy.linalg import norm
from numpy import mean 

def leveling_ur5_plan():
    
    # Initialization
    print "----Initialization..."
    moveit_commander.roscpp_initialize(sys.argv)    #moveit_commander: the python moveit interface
    rospy.init_node('ur5_level',anonymous=True)  #initialize a ros node
    robot = moveit_commander.RobotCommander()   #initialize the robot object
    scene = moveit_commander.PlanningSceneInterface()   #initialize the scence object
    group = moveit_commander.MoveGroupCommander("manipulator")  #the group to be planned

    # Start RViz
    print "----Wait for RViz..."
    rospy.sleep(5)

    # UR configuration initialization
    # Joints Plan
    print "----Move to initial position"
    group_variable_values = group.get_current_joint_values()
    initial_position = [0.0, -1.2, 1.8, -0.6, 1.6, 0.0]
    group_variable_values = list(initial_position)
    group.set_joint_value_target(group_variable_values) #Set a joint target
    initial_plan=group.plan()
    group.execute(initial_plan)
    rospy.sleep(2)  #Wait for RViz to visualize

    # Prepare for Cartesian Plan
    print "----Ready to receive command from LeapMotion"
    waypoints = []
    waypoints.append(group.get_current_pose().pose) # start with the current pose
    base_xyz = [waypoints[0].position.x, waypoints[0].position.y, waypoints[0].position.z ] #Initial position is the basement
    wpose = geometry_msgs.msg.Pose()
    wpose.orientation.w = 1.0 # first orient gripper and move forward (+x)

    # Start Leap Service
    leaper_0 = leaper.leaper(0.1)
    leaper_0.leaper_start()
    print "----Leap Motion is Ready!"

    #target test topic
    pub = rospy.Publisher("current_xyz_target", xyz_targets, queue_size=10)
    rate = rospy.Rate(1)
    current_targets = xyz_targets()

    # Start Plan
    leap_counter = 0
    final_xyz_set = []  # Store some xyz data
    while not rospy.is_shutdown():
        if leaper_0.is_hand_exist():
            final_xyz = leaper_0.get_target_xyz()
            final_xyz_set.append(final_xyz)
            if leap_counter >= 20:
                # Take the mean
                final_xyz = mean(final_xyz_set, axis=0)
                final_xyz_set = []
                leap_counter = 0
                # Prepare for the msg
                current_targets.x = final_xyz[0]
                current_targets.y = final_xyz[1]
                current_targets.z = final_xyz[2]
                current_targets.hand = 1
                # Prepare for the shreshold
                current_xyz = [waypoints[0].position.x, waypoints[0].position.y, waypoints[0].position.z ]
                sign = [final_xyz[0]-current_xyz[0], final_xyz[1]-current_xyz[1], final_xyz[2]-current_xyz[2] ]
                if norm( sign ) >=0.01: # There is a shreshold to eliminate noise to some degree
                    # Cartesian Plan
                    wpose.position.x = base_xyz[0] + final_xyz[0]
                    wpose.position.y = base_xyz[1] + final_xyz[1]
                    wpose.position.z = base_xyz[2] + final_xyz[2]
                    waypoints.append(copy.deepcopy(wpose))
                    (cartesian_plan, fraction) = group.compute_cartesian_path(
                                                waypoints,   # waypoints to follow
                                                0.01,        # eef_step, 1cm
                                                0.0)         # jump_threshold, disabling
                    group.execute(cartesian_plan)

                pub.publish(current_targets)
                rate.sleep()
                waypoints.pop(0)
            else:
                leap_counter += 1

    moveit_commander.roscpp_shutdown()  #When finished, shut down moveit_commander


if __name__=='__main__':
    try:
        leap_ur5_plan()
    except rospy.ROSInterruptException:
        pass

