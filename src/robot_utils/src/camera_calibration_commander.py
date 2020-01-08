#!/usr/bin/python
'''
# Authors:  Filippos Sotiropoulos (filippos.sotiropoulos@gmail.com)
#           Ryan Sandzimier (rsandzimier@gmail.com)

ROS Node for calibrating over head camera relative to robot
'''

import rospy
from sensor_msgs.msg import Joy
from ur_lightweight_driver.msg import Mode, Setpoint, Trajectory
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
import math

import numpy as np
import numpy.matlib as npm

import threading
import tf

CALIBRATION_TRAJECTORIES_JOINT_TOP  =   [[[5.0,[math.radians(16.7),  math.radians(-36.4), math.radians(75.0),  math.radians(-130.0), math.radians(85.5),  math.radians(-22.6)]]],
                                        [[2.0,[math.radians(-17.0), math.radians(-44.0), math.radians(90.7),  math.radians(-150.3), math.radians(85.6),  math.radians(-22.6)]]],
                                        [[2.0,[math.radians(-20.6), math.radians(-64.3), math.radians(129.0),  math.radians(-157.0), math.radians(87.5),  math.radians(-24.5)]]],
                                        [[2.0,[math.radians(24.6), math.radians(-62.3), math.radians(122.8),  math.radians(-147.0), math.radians(85.5),  math.radians(-22.2)]]],
                                        [[2.0,[math.radians(16.7),  math.radians(-36.4), math.radians(75.0),  math.radians(-130.0), math.radians(85.5),  math.radians(-22.6)]]],
                                        [[2.0,[math.radians(-17.0), math.radians(-44.0), math.radians(90.7),  math.radians(-150.3), math.radians(85.6),  math.radians(-22.6)]]],
                                        [[2.0,[math.radians(-20.6), math.radians(-64.3), math.radians(129.0),  math.radians(-157.0), math.radians(87.5),  math.radians(-24.5)]]],
                                        [[2.0,[math.radians(24.6), math.radians(-62.3), math.radians(122.8),  math.radians(-147.0), math.radians(85.5),  math.radians(-22.2)]]]]


CALIBRATION_TRAJECTORIES_JOINT_FRONT=   [[[5.0,[math.radians(14.0),  math.radians(-22.0), math.radians(42.0),  math.radians(-15.0), math.radians(-75.0),  math.radians(0.0)]]],
                                        [[2.0,[math.radians(17.0),  math.radians(-34.0), math.radians(70.0),  math.radians(-22.0), math.radians(-80.0),  math.radians(0.0)]]],
                                        [[2.0,[math.radians(20.0),  math.radians(-38.0), math.radians(80.0),  math.radians(-22.0), math.radians(-80.0),  math.radians(0.0)]]],
                                        [[2.0,[math.radians(20.0),  math.radians(-35.0), math.radians(80.0),  math.radians(-22.0), math.radians(-80.0),  math.radians(0.0)]]],
                                        [[2.0,[math.radians(17.0),  math.radians(-24.0), math.radians(56.0),  math.radians(-9.0), math.radians(-83.0),  math.radians(0.0)]]],
                                        [[2.0,[math.radians(14.0),  math.radians(-22.0), math.radians(42.0),  math.radians(-15.0), math.radians(-75.0),  math.radians(0.0)]]],
                                        [[2.0,[math.radians(17.0),  math.radians(-34.0), math.radians(70.0),  math.radians(-22.0), math.radians(-80.0),  math.radians(0.0)]]],
                                        [[2.0,[math.radians(20.0),  math.radians(-38.0), math.radians(80.0),  math.radians(-22.0), math.radians(-80.0),  math.radians(0.0)]]],
                                        [[2.0,[math.radians(20.0),  math.radians(-35.0), math.radians(80.0),  math.radians(-22.0), math.radians(-80.0),  math.radians(0.0)]]],
                                        [[2.0,[math.radians(17.0),  math.radians(-24.0), math.radians(56.0),  math.radians(-9.0), math.radians(-83.0),  math.radians(0.0)]]]]

def averageQuaternions(Q):
    # Number of quaternions to average
    M = Q.shape[0]

    A = npm.zeros(shape=(4,4))

    for i in range(0,M):
        q = Q[i,:]
        # multiply q with its transposed version q' and add A
        A = np.outer(q,q) + A

    # scale
    A = (1.0/M)*A
    # compute eigenvalues and -vectors
    eigenValues, eigenVectors = np.linalg.eig(A)
    # Sort by largest eigenvalue
    eigenVectors = eigenVectors[:,eigenValues.argsort()[::-1]]
    # return the real part of the largest eigenvector (has only real part)
    return np.real(eigenVectors[:,0].A1)

class JoyMsgManager:
    def __init__(self):
        rospy.init_node('joy_msg_manager', anonymous=True)
        
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()

        self.joy_vals_right = [0.0,0.0]
        self.joy_vals_left = [0.0,0.0]

        self.control_mode   = 0 # this nodes control mode

        self.robot_ready  = False

        self.calibrating = False

        self.time_start_update = 0

        self.sub_joy_right      = rospy.Subscriber('joy_right', Joy, self.cb_joy_right)
        self.sub_robot_ready    = rospy.Subscriber('robot_ready', Bool, self.cb_robot_ready)

        self.pub_mode           = rospy.Publisher('mode', Mode, queue_size= 1)
        self.pub_setpoint       = rospy.Publisher('setpoint', Setpoint, queue_size= 1)
        self.pub_trajectory     = rospy.Publisher('trajectory', Trajectory, queue_size= 1)

        self.time_switch_last   = rospy.get_rostime()


    def cb_robot_ready(self, robot_ready_msg):
        self.robot_ready = robot_ready_msg.data
        if not self.robot_ready and self.control_mode != 0:
            print "STANDBY mode engaged from program halt"
            self.control_mode = 0 

    def cb_joy_right(self, joy):
        if not self.robot_ready:
            print "Cannot use joystick while robot not ready"
            return
        if joy.buttons[0] == 1 and rospy.get_rostime()-self.time_switch_last>rospy.Duration(0,100000000):
            print "STANDBY"
            self.control_mode   = 0
            self.pub_mode.publish(Mode(Mode.MODE_STANDBY))
            self.time_switch_last = rospy.get_rostime()

        if joy.buttons[7] == 1 and rospy.get_rostime()-self.time_switch_last>rospy.Duration(0,100000000):
            print "STARTING CAMERA CALIBRATION - TOP CAMERA"
            self.calibration_procedure("top")
            self.time_switch_last = rospy.get_rostime()

    def calibration_procedure(self, camera):
        if self.calibrating:
            print "Cannot begin calibration while calibration already in progress"
            return
        if camera == "top":
            trajectories = CALIBRATION_TRAJECTORIES_JOINT_TOP
            tf_prefix = "/zed_top"
        elif camera == "front":
            trajectories = CALIBRATION_TRAJECTORIES_JOINT_FRONT
            tf_prefix = "/zed_front"
        else:
            return
        self.calibrating = True

        self.control_mode   = 4

        calibration_translations = []
        calibration_quaternions  = []

        setpoints = []
        n = 0
        for tr in trajectories:
            
            setpoints = []

            for sp in tr:

                sp_msg = Setpoint()
                sp_msg.time = sp[0]
                sp_msg.setpoint = sp[1]
                sp_msg.type = Setpoint.TYPE_JOINT_POSITION
                setpoints.append(sp_msg)
            
            trajectory_msg = Trajectory()
            trajectory_msg.setpoints = setpoints
            self.pub_trajectory.publish(trajectory_msg)

            rospy.sleep(sp[0]+2.0)

            try:
                base_link = tf_prefix+"/base_link"
                last_time = self.listener.getLatestCommonTime(base_link,"tag_0").to_sec()
                i = 0
                while self.listener.getLatestCommonTime(base_link,"tag_0").to_sec() == last_time:
                    rospy.sleep(0.1)
                    i+=1
                    if i >= 20:
                        raise tf.Exception

                (trans,rot) = self.listener.lookupTransform( 'tag_0',base_link,rospy.Time())
                self.broadcaster.sendTransform(trans,rot,rospy.get_rostime(),base_link,"tag_0_cad")
                rospy.sleep(0.2)
                (trans,rot) = self.listener.lookupTransform( 'robot_base',base_link,rospy.Time(0))
                calibration_translations.append(trans)
                calibration_quaternions.append(rot)
                n += 1

            except tf.Exception:
                print "tf error. not included in calibration"

        if n == 0:
            print "Calibration failed. No tf's found"
            return

        calibrated_quaternion_np = averageQuaternions(np.array(calibration_quaternions))
        calibrated_quaternion    = calibrated_quaternion_np.tolist()

        calibrated_translation   = np.mean(np.array(calibration_translations),axis = 0).tolist()
        print "Successfully calibrated using ", n, " tf's"
        print "Translation",calibrated_translation,"Quaternion",calibrated_quaternion
        self.write_to_file(camera, calibrated_translation, calibrated_quaternion)
        self.calibrating = False
    
    def write_to_file(self, camera, trans, rot):
        filename = rospy.get_param("camera_calibration_commander/frames_file", 'src/robot_utils/launch/frames.launch') # Get param, otherwise default
        prefix = "  <arg name=\"tf_camera_" + camera + "_robot_base_args\" default=\""
        suffix = " ".join(map(str, trans)) + " " + " ".join(map(str, rot)) + " robot_base zed_" + camera + "/base_link 10\"/>\n"
        f = open(filename, "r")
        lines = [] 
        for x in f:
            if x.startswith(prefix):
                lines.append(prefix + suffix)
            else:
                lines.append(x)
        f.close() 
        f = open(filename, "w")
        for x in lines:
            f.write(x)
        f.close()
            
if __name__ == '__main__':
    
    jm = JoyMsgManager()
    
    try:
        rospy.spin()

    except rospy.ROSInterruptException:
        pass