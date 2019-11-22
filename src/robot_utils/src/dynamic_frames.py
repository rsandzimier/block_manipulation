#!/usr/bin/python

# Authors:  Filippos Sotiropoulos (fes@mit.edu)
#            Ryan Sandzimier (rsandz@mit.edu)


import rospy
from sensor_msgs.msg import JointState
from ur_rtde.msg import ToolState
import tf
import math

class DynamicFrames:
    def __init__(self):
        rospy.init_node('dynamic_frames', anonymous=True)
        rospy.Subscriber('joint_states', JointState, self.cb_joint_states)
        self.br = tf.TransformBroadcaster()

    def cb_joint_states(self, msg): # Broadcast transform from robot_base to bucket
        self.br.sendTransform((0.0,0.0,0.0993),tf.transformations.quaternion_from_euler(0.0,0.0,msg.position[0],axes="rzxz"),msg.header.stamp,"robot_link1","robot_base")
        self.br.sendTransform((0.0,-0.0946,0.0814),tf.transformations.quaternion_from_euler(0.0,math.pi/2,msg.position[1],axes="rzxz"),msg.header.stamp,"robot_link2","robot_link1")
        self.br.sendTransform((-0.6127,0.0,0.018),tf.transformations.quaternion_from_euler(0.0,0.0,msg.position[2],axes="rzxz"),msg.header.stamp,"robot_link3","robot_link2")
        self.br.sendTransform((-0.57155,0.0,-0.0169),tf.transformations.quaternion_from_euler(0.0,0.0,msg.position[3],axes="rzxz"),msg.header.stamp,"robot_link4","robot_link3")
        self.br.sendTransform((0.0,-0.0569,0.07845),tf.transformations.quaternion_from_euler(0.0,math.pi/2,msg.position[4],axes="rzxz"),msg.header.stamp,"robot_link5","robot_link4")
        self.br.sendTransform((0.0,0.0569,0.06295),tf.transformations.quaternion_from_euler(0.0,-math.pi/2,msg.position[5],axes="rzxz"),msg.header.stamp,"robot_link6","robot_link5")
        self.br.sendTransform((0.0,0.0,0.05965),tf.transformations.quaternion_from_euler(0.0,0.0,0.0),msg.header.stamp,"bucket","robot_link6")

if __name__ == '__main__':
    df = DynamicFrames()
    try:              
        rospy.spin()

    except rospy.ROSInterruptException:
        pass