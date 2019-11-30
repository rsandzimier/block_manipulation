#!/usr/bin/python
'''
Created on Feb 21, 2017

# Authors:  Filippos Sotiropoulos (fes@mit.edu)
#           Ryan Sandzimier (rsandz@mit.edu)
'''

import rospy
from sensor_msgs.msg import Joy
from ur_lightweight_driver.msg import Mode, Setpoint, Trajectory
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import JointState
from robot_utils.srv import PickPlace, Push, Home  

import math
import numpy as np
import threading

TEST_TRAJECTORY =    [[5.0 ,[0.0, -math.pi/2, 0.0       , -math.pi/2, 0.0, 0.0]],
                     [10.0 ,[0.0, -math.pi/4, -math.pi/2, 1.0       , 0.0, 1.0]],
                     [15.0,[0.0, -math.pi/2, 0.0       , 2.0       , 0.0, 2.0]]]

# DRAG, SCOOP, AND DUMP TEST TRAJECTORY
TEST_TRAJECTORY_CARTESIAN =    [[4.0,[-0.800, -0.400, 0.500, math.pi, 0.0, math.pi/2]],
                               [8.0,[-1.100, -0.400, 0.075, math.pi, 0.0, math.pi/2]],
                               [12.0,[-1.000, -0.400, 0.050, math.pi, 0.0, math.pi/2]],
                               [16.0,[-0.800, -0.400, 0.050, math.pi, 0.0, math.pi/2]],
                               [20.0,[-0.600, -0.400, 0.100, math.pi/2, 0.0, math.pi/2]],
                               [24.0,[-0.800, -0.400, 0.300, math.pi/2, 0.0, math.pi/2]],
                               [28.0,[-0.800, 0.200, 0.300, math.pi/2, 0.0, math.pi/2]],
                               [32.0,[-0.800, 0.200, 0.300, math.pi, 0.0, math.pi/2]],
                               [36.0,[-0.800, 0.200, 0.500, math.pi, 0.0, math.pi/2]]]

TEST_TRAJECTORY_CARTESIAN =   [[2.0,[-1.000, -0.500, 0.125, math.pi, 0.0, math.pi/2]],
                                        [4.0,[-0.800, -0.500, 0.125, math.pi, 0.0, math.pi/2]],
                                        [6.0,[-0.800, -0.500, 0.125, math.pi/2, 0.0, math.pi/2]]]


TEST_TRAJECTORY_CARTESIAN =   [[5.0,[-0.800, 0.0, 0.350, math.pi, 0.0, 0.0]],
                                [10.0,[-0.800, 0.0, 0.200, math.pi, 0.0, 0.0]],
                                [15.0,[-0.800, 0.0, 0.350, math.pi, 0.0, 0.0]],]   
TEST_TRAJECTORY_CARTESIAN =   [[2.0,[-0.804, -0.014, 0.350, math.pi, 0.0, 0.0, 0.0]],
                                [4.0,[-0.804, -0.014, 0.180, math.pi, 0.0, 0.0, 0.0]],
                                [5.0,[-0.804, -0.014, 0.180, math.pi, 0.0, 0.0, 100.0]],
                                [7.0,[-0.804, -0.014, 0.350, math.pi, 0.0, 0.0, 100.0]],
                                [10.0,[-0.804, 0.200, 0.350, math.pi, 0.0, 0.0, 100.0]],
                                [12.0,[-0.804, 0.200, 0.180, math.pi, 0.0, 0.0, 100.0]],
                                [13.0,[-0.804, 0.200, 0.180, math.pi, 0.0, 0.0, 0.0]],
                                [15.0,[-0.804, 0.200, 0.350, math.pi, 0.0, 0.0, 0.0]],]                                

class JoyMsgManager:
    def __init__(self):
        rospy.init_node('joy_msg_manager', anonymous=True)
        
        self.joy_vals_right = [0.0,0.0]
        self.joy_vals_left = [0.0,0.0]

        self.control_mode   = 0 # this nodes control mode

        self.robot_ready  = False

        self.time_start_update = 0  

        self.sub_joy_right    = rospy.Subscriber('joy_right', Joy, self.cb_joy_right)
        self.sub_joy_left     = rospy.Subscriber('joy_left', Joy, self.cb_joy_left)
        self.sub_robot_ready  = rospy.Subscriber('robot_ready', Bool, self.cb_robot_ready)

        self.pub_mode   = rospy.Publisher('mode', Mode, queue_size= 1)
        self.pub_setpoint   = rospy.Publisher('setpoint', Setpoint, queue_size= 1)
        self.pub_trajectory   = rospy.Publisher('trajectory', Trajectory, queue_size= 1)

        self.time_switch_last = rospy.get_rostime()

        rospy.Timer(rospy.Duration(0.002), self.cb_publish)

    def cb_robot_ready(self, robot_ready_msg):
        self.robot_ready = robot_ready_msg.data
        if not self.robot_ready and self.control_mode != 0:
            print "STANDBY mode engaged from program halt"
            self.control_mode = 0 

    def cb_joy_right(self, joy):
        if joy.buttons[0] == 1 and rospy.get_rostime()-self.time_switch_last>rospy.Duration(0,100000000):
            # POSITION MODE
            print "STANDBY"
            self.control_mode   = 0
            self.pub_mode.publish(Mode(Mode.MODE_STANDBY))
            self.time_switch_last = rospy.get_rostime()

        if joy.buttons[1] == 1 and rospy.get_rostime()-self.time_switch_last>rospy.Duration(0,100000000):
            # POSITION MODE
            print "VELOCITY MODE"
            self.control_mode   = 3
            self.time_switch_last = rospy.get_rostime()

        if joy.buttons[5] == 1 and rospy.get_rostime()-self.time_switch_last>rospy.Duration(0,100000000):
            # SERVO J TEST MODE
            print "Disbled sinusoid test" # Need to add check to make sure set point is close enough to current position
            return
            print "SINUSOID TEST"
            self.control_mode   = 2
            self.timer          = 0.0
            self.time_switch_last = rospy.get_rostime()

        if joy.buttons[6] == 1 and rospy.get_rostime()-self.time_switch_last>rospy.Duration(0,100000000):
            print "JOINT TRAJECTORY PUBLISHER TEST"
            self.control_mode   = 1
            setpoints = []
            for sp in TEST_TRAJECTORY:
                sp_msg = Setpoint()
                sp_msg.time = sp[0]
                sp_msg.setpoint = sp[1]
                sp_msg.type = Setpoint.TYPE_JOINT_POSITION
                setpoints.append(sp_msg)
            trajectory_msg = Trajectory()
            trajectory_msg.setpoints = setpoints
            self.pub_trajectory.publish(trajectory_msg)
            self.time_switch_last = rospy.get_rostime()
        if joy.buttons[7] == 1 and rospy.get_rostime()-self.time_switch_last>rospy.Duration(0,100000000):

            print "CARTESIAN TRAJECTORY PUBLISHER TEST"
            self.control_mode   = 4
            setpoints = []
            for sp in TEST_TRAJECTORY_CARTESIAN:
                sp_msg = Setpoint()
                sp_msg.time = sp[0]
                sp_msg.setpoint = sp[1]
                sp_msg.type = Setpoint.TYPE_CARTESIAN_POSITION
                setpoints.append(sp_msg)
            trajectory_msg = Trajectory()
            trajectory_msg.setpoints = setpoints
            self.pub_trajectory.publish(trajectory_msg)
            self.time_switch_last = rospy.get_rostime()

        if joy.buttons[8] == 1 and rospy.get_rostime()-self.time_switch_last>rospy.Duration(0,100000000):
            # IMPEDANCE MODE
            print "IMPEDANCE SETPOINT MODE"
            self.control_mode   = 5
            self.setpoint = [-1.0,-0.50,0.125,2.221,2.221,0.0]
            self.time_switch_last = rospy.get_rostime()

        if joy.buttons[9] == 1 and rospy.get_rostime()-self.time_switch_last>rospy.Duration(0,100000000):
            # IMPEDANCE MODE
            print "IMPEDANCE TRAJECTORY MODE"
            self.control_mode   = 6
            setpoints = []
            for sp in TEST_TRAJECTORY_CARTESIAN:
                sp_msg = Setpoint()
                sp_msg.time = sp[0]
                sp_msg.setpoint = sp[1]
                sp_msg.type = Setpoint.TYPE_CARTESIAN_POSITION_IMPEDANCE
                setpoints.append(sp_msg)
            trajectory_msg = Trajectory()
            trajectory_msg.setpoints = setpoints
            self.pub_trajectory.publish(trajectory_msg)
            self.time_switch_last = rospy.get_rostime()

        if joy.buttons[10] == 1 and rospy.get_rostime()-self.time_switch_last>rospy.Duration(0,100000000):
            self.control_mode   = 7
            home = rospy.ServiceProxy('home', Home)
            pick_place = rospy.ServiceProxy('pick_place', PickPlace)
            push = rospy.ServiceProxy('push', Push)

            home()
            pick_place(Pose2D(-0.804,-0.014,0.0),Pose2D(-0.704,0.05,np.pi/4))
            push(Pose2D(-0.704,0.05,np.pi/4),Float32(0.15))
            pick_place(Pose2D(-0.704+0.15*np.cos(np.pi/4),0.05+0.15*np.sin(np.pi/4),np.pi/4),Pose2D(-0.804,-0.014,0.0))
            home()

            self.time_switch_last = rospy.get_rostime()

        self.joy_vals_right[0] = joy.axes[0]
        self.joy_vals_right[1] = joy.axes[1]
    def cb_joy_left(self, joy):
        self.joy_vals_left[0] = joy.axes[0]
        self.joy_vals_left[1] = joy.axes[1]
            
    def cb_publish(self, event):
        if self.control_mode == 2:
            setpoint = [0.0]*6
            setpoint[0] = 0.3*math.sin(0.5*self.timer)
            setpoint[1] = 0.3*math.sin(0.6*self.timer) - math.pi/2
            setpoint[2] = 0.3*math.sin(0.7*self.timer)
            setpoint[3] = 0.75*math.sin(1*self.timer) - math.pi/2
            setpoint[4] = 0.75*math.sin(1*self.timer)
            setpoint[5] = 0.75*math.sin(1*self.timer)
            self.timer = self.timer + 0.002
            setp_msg = Setpoint()
            setp_msg.setpoint = setpoint
            setp_msg.type = Setpoint.TYPE_JOINT_POSITION
            self.pub_setpoint.publish(setp_msg)
        elif self.control_mode == 3:
            setpoint = [0.0]*6
            setpoint[0] = self.joy_vals_left[0]*0.25
            setpoint[1] = self.joy_vals_right[1]*0.25
            setpoint[2] = self.joy_vals_left[1]*0.25
            setpoint[3] = self.joy_vals_right[0]*0.25
            setp_msg = Setpoint()
            setp_msg.setpoint = setpoint
            setp_msg.type = Setpoint.TYPE_JOINT_VELOCITY
            self.pub_setpoint.publish(setp_msg)
        elif self.control_mode == 5:
            self.setpoint[0] = self.setpoint[0] - self.joy_vals_left[1]*0.1*0.002
            self.setpoint[2] = self.setpoint[2] + self.joy_vals_right[1]*0.1*0.002
            setp_msg = Setpoint()
            setp_msg.setpoint = self.setpoint
            setp_msg.type = Setpoint.TYPE_CARTESIAN_POSITION_IMPEDANCE
            self.pub_setpoint.publish(setp_msg)
   
if __name__ == '__main__':
    jm = JoyMsgManager()
    
    try:              
        rospy.spin()

    except rospy.ROSInterruptException:
        pass