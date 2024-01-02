#! /usr/bin/env python
'''
- how to actually activate the robot 
    turn on the robot
    run the example ROS command (cd /src/pick_demo)) and then run code . to see the space - note that Eric has a difference launch script i can use
    program robot and select the file
    note: should see the correct starting position for the robot on rviz
    click play
    source workspace
    rosrun ur5_control move_test.py
- can manually move the arm to different positions, take note of the arm values and set those as poses in code.
- then hardcode the movements to these predetermined positions, close gripper on object, move to another position, drop
- all this while microphones are connected to the table and near gripper to record the sounds
'''
import rospy
import numpy as np
from geometry_msgs.msg import Pose

from ur5_moveit import UR5Moveit 

if __name__ == "__main__":
    rospy.init_node("ur5_movement_test")

    ur5 = UR5Moveit()
    
    # initialise planning scene

    # Fetch current end-effector pose
    current_pose = ur5.arm.get_current_pose().pose

    # Move the robot's effector slightly in the positive x-direction
    translation = 0.05  # Adjust this value as needed
    new_pose = Pose()
    new_pose.position.x = current_pose.position.x + translation
    new_pose.position.y = current_pose.position.y
    new_pose.position.z = current_pose.position.z
    new_pose.orientation = current_pose.orientation

    # Move to the new Cartesian goal
    ur5.move_to_cartesian_goal(new_pose) # also plans and executes

    
    #.execute or .run function

    rospy.spin()
