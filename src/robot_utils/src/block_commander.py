#!/usr/bin/python
'''
Created on Feb 21, 2017

# Authors:  Filippos Sotiropoulos (fes@mit.edu)
#           Ryan Sandzimier (rsandz@mit.edu)
'''

import rospy
from ur_lightweight_driver.msg import Mode, Setpoint, Trajectory
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose2D
from robot_utils.srv import PickPlace, Push, Home  
import math
import numpy as np
import threading

class BlockActionCommander:
    def __init__(self):
        rospy.init_node('block_action_commander', anonymous=True)
        
        self.pub_trajectory   = rospy.Publisher('trajectory', Trajectory, queue_size= 1)

        self.pick_place = rospy.Service('pick_place', PickPlace, self.pick_place)
        self.push = rospy.Service('push', Push, self.push)
        self.home = rospy.Service('home', Home, self.home)

        self.home_pose = np.array([-0.600,-0.500,0.17,0.0,0.0])
        self.current_pose = None
        self.move_time_unknown_pose = 10.0 # Move time to use when current pose is not known

        self.gripper_open_pos = 0.0
        self.gripper_close_pos = 100.0
        self.gripper_push_pos = 60.0 

        self.min_move_time = 0.5

        self.max_trans_vel = 0.1
        self.max_rot_vel = 1.0
        self.max_gripper_vel = 100.0

        self.z_clear = 0.07
        self.z_push = 0.0
        self.traj_time = 0.0

        self.coordinate_sys_offset = np.array([0.0,0.0,0.180,0.0,0.0,0.0,0.0])

        self.block_half_width = 0.0254
        self.gripper_half_width = 0.012
        self.push_buffer = 0.01

    def pick_place(self, req):
        start = req.start_pose
        end = req.end_pose

        setpoints = []
        self.traj_time = 0.0
        above = [start.x,start.y,self.z_clear,start.theta,self.gripper_open_pos]
        setpoints.append(self.calc_setpoint(self.current_pose,above))
        pre_grasp = [start.x,start.y,0.0,start.theta,self.gripper_open_pos]
        setpoints.append(self.calc_setpoint(above,pre_grasp))
        grasp = [start.x,start.y,0.0,start.theta,self.gripper_close_pos]
        setpoints.append(self.calc_setpoint(pre_grasp,grasp))
        up = [start.x,start.y,self.z_clear,start.theta,self.gripper_close_pos]
        setpoints.append(self.calc_setpoint(grasp,up))
        above_final = [end.x,end.y,self.z_clear,end.theta,self.gripper_close_pos]
        setpoints.append(self.calc_setpoint(up,above_final))
        pre_release = [end.x,end.y,0.0,end.theta,self.gripper_close_pos]
        setpoints.append(self.calc_setpoint(above_final,pre_release))
        release = [end.x,end.y,0.0,end.theta,self.gripper_open_pos]
        setpoints.append(self.calc_setpoint(pre_release,release))
        final = [end.x,end.y,self.z_clear,end.theta,self.gripper_open_pos]
        setpoints.append(self.calc_setpoint(release,final))

        trajectory_msg = Trajectory()
        trajectory_msg.setpoints = setpoints
        self.pub_trajectory.publish(trajectory_msg)
        self.current_pose = final
        return rospy.wait_for_message('/trajectory_finished', Bool)

    def push(self, req):
        start = req.start_pose
        dist = req.push_distance.data

        D_pre = -self.block_half_width-self.gripper_half_width-self.push_buffer
        D_push = D_pre + self.push_buffer + dist
        D_post = D_push - self.push_buffer
        print start, dist, D_pre, D_push, D_post, np.cos(start.theta), np.sin(start.theta)
        gripper_angle = start.theta + np.pi/2

        setpoints = []
        self.traj_time = 0.0
        above = [start.x+D_pre*np.cos(start.theta),start.y+D_pre*np.sin(start.theta),self.z_clear,gripper_angle,self.gripper_push_pos]
        setpoints.append(self.calc_setpoint(self.current_pose,above))
        pre_push = [start.x+D_pre*np.cos(start.theta),start.y+D_pre*np.sin(start.theta),self.z_push,gripper_angle,self.gripper_push_pos]
        setpoints.append(self.calc_setpoint(above,pre_push))
        push = [start.x+D_push*np.cos(start.theta),start.y+D_push*np.sin(start.theta),self.z_push,gripper_angle,self.gripper_push_pos]
        setpoints.append(self.calc_setpoint(pre_push,push))
        back_off = [start.x+D_post*np.cos(start.theta),start.y+D_push*np.sin(start.theta),self.z_push,gripper_angle,self.gripper_push_pos]
        setpoints.append(self.calc_setpoint(push,back_off))
        final = [start.x+D_post*np.cos(start.theta),start.y+D_push*np.sin(start.theta),self.z_clear,gripper_angle,self.gripper_push_pos]
        setpoints.append(self.calc_setpoint(back_off,final))

        trajectory_msg = Trajectory()
        trajectory_msg.setpoints = setpoints
        self.pub_trajectory.publish(trajectory_msg)
        self.current_pose = final
        return rospy.wait_for_message('/trajectory_finished', Bool)

    def home(self, req):
        setpoints = []
        self.traj_time = 0.0
        setpoints.append(self.calc_setpoint(self.current_pose,self.home_pose))

        trajectory_msg = Trajectory()
        trajectory_msg.setpoints = setpoints
        self.pub_trajectory.publish(trajectory_msg)
        self.current_pose = self.home_pose
        return rospy.wait_for_message('/trajectory_finished', Bool)

    def calc_setpoint(self,s1,s2):
        s2 = np.array(s2)
        sp_msg = Setpoint()
        if s1 is None:
            self.traj_time += self.move_time_unknown_pose
        else:
            s1 = np.array(s1)
            self.traj_time += max(np.linalg.norm(s1[0:3]-s2[0:3])/self.max_trans_vel,np.abs(s1[3]-s2[3])/self.max_rot_vel,np.abs(s1[4]-s2[4])/self.max_gripper_vel,self.min_move_time)
        sp_msg.time = self.traj_time
        sp_msg.setpoint = np.array([s2[0],s2[1],s2[2],np.pi,0.0,s2[3],s2[4]])+self.coordinate_sys_offset
        sp_msg.type = Setpoint.TYPE_CARTESIAN_POSITION
        return sp_msg
   
if __name__ == '__main__':
    bac = BlockActionCommander()
    
    try:              
        rospy.spin()

    except rospy.ROSInterruptException:
        pass