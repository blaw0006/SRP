#! /usr/bin/env python

import rospy
import moveit_commander
import numpy as np
import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Constraints, JointConstraint
#from slip_manipulation.get_tf_helper import *
from moveit_msgs.srv import GetCartesianPath, GetCartesianPathRequest
from moveit_msgs.msg import MoveItErrorCodes

class UR5Moveit():
    
    def __init__(self):

        # initialise moveit planning scene
        self.scene = moveit_commander.PlanningSceneInterface(synchronous=True)
        self.table_size = [2, 2, 0.87]

        self.pose_publisher = rospy.Publisher('display_pose', PoseStamped, queue_size=1)
        
        self.arm = moveit_commander.MoveGroupCommander('manipulator')
      
        self.arm.set_planner_id("RRTStar") # /home/acrv/HRIGroupAdmin/example_ros_ws/src/universal_robot/ur5_moveit_config/config/ompl_planning.yaml

        # print(moveit_commander.RobotCommander().get_link_names())

        self.init_planning_scene()

        # initialise joint constraints to prevent collision with camera mount
        self.init_moveit_constraints()

        # set up service proxy for cartesian path planning
        rospy.wait_for_service('/compute_cartesian_path', timeout=rospy.Duration(10))
        self.cartesian_srv = rospy.ServiceProxy('/compute_cartesian_path', GetCartesianPath)

        # define important poses
        # self.arm.set_named_target("up") # go to up position if not already there
        self.start_pose = {
            'shoulder_pan_joint': -np.pi/2,
            'shoulder_lift_joint': (-94) * np.pi/180,
            'elbow_joint': (-65) * np.pi/180,
            'wrist_1_joint': (-111) * np.pi/180,
            'wrist_2_joint': np.pi/2,
            'wrist_3_joint': np.pi/2
        }

        # move to start pose
        self.move_to_joints_goal(self.start_pose)

    def init_planning_scene(self):
        # add table collision object
        box_pose = PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose.position.x = 0
        box_pose.pose.position.y = 0
        
        box_pose.pose.position.z = box_pose.pose.position.z - .44 # shift by table size/2
        box_name = "table"
        self.scene.add_box(box_name, box_pose, size=self.table_size)

    def init_moveit_constraints(self):
        # clear existing constraints
        self.arm.clear_path_constraints()
        
        # self.camera_constraints = Constraints()
        # self.camera_constraints.name = 'camera'
        
        # joint_constraint = JointConstraint()
        # joint_constraint.joint_name = 'wrist_2_joint'
        # joint_constraint.position = 0
        # joint_constraint.tolerance_above = 2.55
        # joint_constraint.tolerance_below = 2.55
        # joint_constraint.weight = 1
        
        # self.camera_constraints.joint_constraints.append(joint_constraint)

        # self.arm.set_path_constraints(self.camera_constraints)
        

    def move_to_joints_goal(self, goal_pose):
        self.arm.set_joint_value_target(goal_pose)

        plan = self.arm.plan()

        raw_input('Check Rviz for joints plan, press enter to execute')

        self.arm.execute(plan)

    def move_to_ee_goal(self, goal_pose):
        self.arm.set_pose_target(goal_pose)
        
        plan = self.arm.plan()
        
        raw_input('Check Rviz for ee plan, press enter to execute')

        self.arm.execute(plan)
        
    def move_to_cartesian_goal(self, goal_pose):
        plan, _ = self.arm.compute_cartesian_path([goal_pose], # waypoints to follow
										        0.01,       # eef_step  
										        0.0)        # jump_threshold  
        
        raw_input('Check Rviz for cartesian plan, press enter to execute')

        self.arm.execute(plan, wait=True)

    def display_pose(self, pose):
        self.pose_publisher.publish(pose)
        
    def compute_cartesian_path_with_start_state(self, start_state, waypoints, max_step, jump_threshold):
        '''
        :param start_state: starting state to plan from
        :type start_state: moveit_msgs.msg.RobotoState
        :param waypoints: a sequence of waypoints to plan through
        :type waypoints: a list of geometry_msgs.msg.Pose
        '''
        req = GetCartesianPathRequest()
        req.header.frame_id = 'base_link'
        req.header.stamp = rospy.Time.now()
        req.start_state = start_state
        req.group_name = 'manipulator'
        req.link_name = self.arm.get_end_effector_link()
        req.waypoints = waypoints
        req.max_step = max_step
        req.jump_threshold = jump_threshold
        req.avoid_collisions = True
        
        try:
            res = self.cartesian_srv.call(req)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            
        if res.error_code.val == MoveItErrorCodes.SUCCESS:
            return (res.solution, res.fraction)
        else:
            return (None, -1.0)

if __name__ == "__main__":
    pass
