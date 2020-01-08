#!/usr/bin/python
'''
# Authors:  Filippos Sotiropoulos (filippos.sotiropoulos@gmail.com)
#           Ryan Sandzimier (rsandzimier@gmail.com)

ROS Node that accepts joystick inputs to initiate planner and calls services provided by block_commander.py
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
import tf

import blocks_environment as env                       

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

        self.listener = tf.TransformListener()

    def cb_robot_ready(self, robot_ready_msg):
        self.robot_ready = robot_ready_msg.data
        if not self.robot_ready and self.control_mode != 0:
            print "STANDBY mode engaged from program halt"
            self.control_mode = 0 

    def cb_joy_right(self, joy):
        if joy.buttons[0] == 1 and rospy.get_rostime()-self.time_switch_last>rospy.Duration(0,100000000):
            print "STANDBY"
            self.control_mode   = 0
            self.pub_mode.publish(Mode(Mode.MODE_STANDBY))
            self.time_switch_last = rospy.get_rostime()

        if joy.buttons[10] == 1 and rospy.get_rostime()-self.time_switch_last>rospy.Duration(0,100000000):
            self.control_mode   = 7
            home = rospy.ServiceProxy('home', Home)
            pick_place = rospy.ServiceProxy('pick_place', PickPlace)
            push = rospy.ServiceProxy('push', Push)

            home()
            start = []
            blocks = [0,1,2,3,4,5,6,7,8]
            for blk in blocks:
                (trans,rot) = self.listener.lookupTransform( 'robot_base','tag_'+str(blk+1),rospy.Time())
                rot_eul = tf.transformations.euler_from_quaternion(rot)
                start.append([trans[0]+0.800,trans[1],rot_eul[2]])

            P = env.Planner(start=start)
            actions,states = P.plrs.startplRS()
            for a,s in zip(actions,states[:-1]):
                if type(a) == env.Planner.PickPlaceAction:
                    print ("Pick",a.start.block_id,"at", (a.start.x,a.start.y,a.start.theta),"and place at",(a.end.x,a.end.y,a.end.theta))
                    pick_place(Pose2D(a.start.x,a.start.y,a.start.theta+a.grasp_offset_90*np.pi/2),Pose2D(a.end.x,a.end.y,a.end.theta+a.grasp_offset_90*np.pi/2))
                elif type(a) == env.Planner.PushAction:
                    print ("Push",a.start.block_id,"at",(a.start.x,a.start.y,a.start.theta),"in direction",a.direction,"for distance",a.distance)
                    push(Pose2D(a.start.x,a.start.y,a.direction),Float32(a.distance))
            home()

            self.time_switch_last = rospy.get_rostime()

        self.joy_vals_right[0] = joy.axes[0]
        self.joy_vals_right[1] = joy.axes[1]
        
    def cb_joy_left(self, joy):
        self.joy_vals_left[0] = joy.axes[0]
        self.joy_vals_left[1] = joy.axes[1]
            
if __name__ == '__main__':
    jm = JoyMsgManager()
    
    try:              
        rospy.spin()

    except rospy.ROSInterruptException:
        pass