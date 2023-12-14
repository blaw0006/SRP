#! /usr/bin/env python

import rospy
import numpy as np
import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
from robotiq.robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from robotiq.robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input  as inputMsg
from geometry_msgs.msg import WrenchStamped
from papillarray_ros_v2.msg import SensorState
from std_msgs.msg import Bool
import csv

class SensorisedGripper():
    def __init__(self):
        # set up gripper subscriber and publisher
        self.gripper_sub = rospy.Subscriber('/Robotiq2FGripperRobotInput', 
            inputMsg.Robotiq2FGripper_robot_input, self.gripper_callback)
        self.gripper_pub = rospy.Publisher('/Robotiq2FGripperRobotOutput', 
            outputMsg.Robotiq2FGripper_robot_output, queue_size=1)

        # max gripper width 180
        self.grip_width = 0
        self.grip_bound = 240   # gripper width that fully closes the grippers when tactile sensors are attached
        self.grip_dist = 0.074 # m
        self.grip_inc = self.grip_bound / self.grip_dist
        
        # set up tactile sensor subscribers
        self.tac0_data = SensorState()
        self.tac1_data = SensorState()

        self.tac0_data_arr = [['seq','gfX','gfY','gfZ','gtX','gtY','gtZ']]
        self.tac1_data_arr = [['seq','gfX','gfY','gfZ','gtX','gtY','gtZ']]

        self.tac0_sub = rospy.Subscriber('/hub_0/sensor_0', SensorState, self.sensor_callback, 0)
        self.tac1_sub = rospy.Subscriber('/hub_0/sensor_1', SensorState, self.sensor_callback, 1)
        
        self.tac0_contact = 0
        self.tac1_contact = 0

        # set up FT 300 sensor subscriber
        self.fts_data = WrenchStamped()
        self.fts_data_arr = [['Fx','Fy','Fz','Mx','My','Mz']]
        
        self.fts_sub = rospy.Subscriber('/robotiq_ft_wrench', WrenchStamped, self.ftSensor_callback)

        # wait for subscribers to initialise and update data in class parameters
        rospy.sleep(1)
        
        self.safety_bound = 20
        self.max_grip_width = 255
        self.fric_coef = 0.15

    def send_gripper_command(self, commandName="deactivate", grip_width=None):
        '''
        Method that sends commands to the gripper by publishing to the /Robotiq2FGripperRobotOutput topic
        
        COMMAND OUTPUT registers
        * rACT: activation bit for the gripper
            0 to deactivate, 1 to activate gripper
        * rGTO: signal to move the gripper to configuration specified by the other registers
            0 to stop, 1 to go to requested position
        * rPR: 8 bit position request
            0x00 is fully open, 0xFF (255) is fully closed
        * rSP: 8 bit Gripper closing/opening speed
            0x00 is minimum speed, 0xFF is maximum speed
        * rFR: final grip force of the closed gripper. Fixes max current sent to the motor while in motion. If
        current limit is exceeded, fingers stop and trigger object detection
            0x00 for min force, 0xFF for Maximum force 
        
        
        * rATR: automatic release routine action that opens gripper fingers to max width as emergency release - overrides all other commands
            0 for normal operation, 1 for emergency auto release
        * rARD: auto release direction
            0 for closing, 1 for opening 
        
        'commands' Starts with all zeros, but lets be careful in case of future changes
        shitty implementation of manual setting of grip width, feed None into commandName to use
        '''
        command = outputMsg.Robotiq2FGripper_robot_output()
        if (grip_width is not None) and (self.max_grip_width >= grip_width >= 0):
            command.rACT = 1
            command.rGTO = 1
            command.rPR = grip_width
            command.rSP  = 50
            command.rFR  = 150
        elif commandName=="deactivate":
            command.rACT = 0
            command.rGTO = 0
            command.rSP  = 50
            command.rGTO = 0
            command.rSP  = 50
            command.rFR  = 150
        elif commandName=="open":
            command.rACT = 1
            command.rGTO = 1
            command.rPR = 0
            command.rSP  = 50
            command.rFR  = 150
        elif commandName=="close":
            command.rACT = 1
            command.rGTO = 1
            command.rPR = 255
            command.rSP  = 50
            command.rFR  = 150
        
        # Command grippertac0_data
        self.gripper_pub.publish(command)

    def sensor_callback(self, data, sensor_num):
        '''
        Callback function for handling data received from tactile sensor
        '''
        # record all sensor data
        if sensor_num == 0:
            self.tac0_data = data
            # if self.tac0_data.is_contact:
            #     temp = [self.tac0_data.header.seq, self.tac0_data.gfX, self.tac0_data.gfY, self.tac0_data.gfZ,
            #     self.tac0_data.gtX, self.tac0_data.gtY, self.tac0_data.gtZ]
            #     self.tac0_data_arr.append(temp)

        else:
            self.tac1_data = data
            # if self.tac1_data.is_contact:
            #     temp = [self.tac1_data.header.seq, self.tac1_data.gfX, self.tac1_data.gfY, self.tac1_data.gfZ,
            #     self.tac1_data.gtX, self.tac1_data.gtY, self.tac1_data.gtZ]
            #     self.tac1_data_arr.append(temp)

    def gripper_callback(self, data):
        '''
        Callback function for recording gripper width data 
        Takes PR (position request) from inputMsg.Robotiq2FGripper_robot_input.gPR
        '''
        # record width of gripper
        self.grip_width = data.gPR

    def ftSensor_callback(self, data):
        '''
        Callback funcion that handles ft sensor data        
        '''
        self.fts_data = data
        temp = [self.fts_data.wrench.force.x, self.fts_data.wrench.force.y, self.fts_data.wrench.force.z, 
                self.fts_data.wrench.torque.x, self.fts_data.wrench.torque.y, self.fts_data.wrench.torque.z]
        self.fts_data_arr.append(temp)

    def manual_grip(self):
        '''
        Manually sets the gripper width based on user input + checks for tactile sensor contact
        '''
        print("current width: " + str(self.grip_width) + "\n")

        # manually enter grip width
        self.grip_width = int(raw_input("enter gripper width value\n"))

        if self.grip_width == -1:
            self.save_data()
            exit()

        self.send_gripper_command(None, self.grip_width)

        # check contact
        rospy.sleep(1) # pauses execution for one second
        print("\n")
        if(self.tac0_data.is_contact and self.tac1_data.is_contact):
            print("both in contact")
        else:
            print("no contact")

        print("\n----------------------------------------")

        # wait for gripper to move
        rospy.sleep(1)

    def touch_object(self, box_dim, box_weight):
        '''
        
        '''
        # slowly tighten gripper until tactile sensors report contact
        
        both_contact = 0 # flag tracking whether both sensors have made contact
        fg = 9.8 * box_weight # gravitational force
        
        # Bool that determines whether the object has a long edge - helps calculate dimensions of the box
        long = rospy.wait_for_message('/slip_manipulation/is_long_edge', Bool, timeout=rospy.Duration(10))
        
        if long.data:
            # print("long is true")
            base_dim = box_dim[0]
            height_dim = box_dim[1]
        else:
            # print("long is false")
            base_dim = box_dim[1]
            height_dim = box_dim[0]
        init_ft_force = fg/2 * np.sin(np.pi/2 - np.arctan(height_dim/base_dim)) * np.cos(np.arctan(height_dim/base_dim))
        
        # initialise gripper width based on box's heigh dim
        init_grip_width =  self.max_grip_width - int(self.grip_inc * box_dim[2])
        print('init grip width', init_grip_width)
        self.send_gripper_command(None, init_grip_width) # send gripper command to set initial width
        rospy.sleep(2) # pauses for two seconds to allow gripper to move

        # loop that gradually tightens gripper until both tactile sensors report contact or until safety bound reached
        # NOTE: will have to do this but until reported sound > given threshold or safety bound reached
        while not both_contact:
            # increment grip tightness
            print('grip width', self.grip_width)
            if self.grip_width < init_grip_width + self.safety_bound:
                self.grip_width = self.grip_width + 1
                self.send_gripper_command(None, self.grip_width)
                rospy.sleep(0.2)
            else: 
                rospy.logerr("Reached safety bound!")
                return None

            # update contact bool
            total_force = self.tac0_data.gfZ + self.tac1_data.gfZ
            print(total_force)
            if self.tac0_data.is_contact and self.tac1_data.is_contact and self.fric_coef * total_force > init_ft_force:
                both_contact = 1

        print("Grasped object.")
        return init_grip_width

    def save_data(self):
        '''
        Saves recorded data from ft sensor, tactile sensor 0 and tactile sensor 1 into CSV files
        
        NOTE: Will have to do the same for each contact mic that is used
        '''
        with open("./src/tactile_data/scripts/fts_data.csv", "wb") as f:
            writer = csv.writer(f)
            writer.writerows(self.fts_data_arr)

        with open("./src/tactile_data/scripts/tac0_data.csv", "wb") as f:
            writer = csv.writer(f)
            writer.writerows(self.tac0_data_arr)
        
        with open("./src/tactile_data/scripts/tac1_data.csv", "wb") as f:
            writer = csv.writer(f)
            writer.writerows(self.tac1_data_arr)

if __name__ == "__main__":
    pass
    