#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from mav_msgs.msg import RollPitchYawrateThrust
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CompressedImage
import numpy as np
import time, sys
import tf
import cv2
from mpctr import Mpc
from detect_marker import detect_markers

class ardrone_ctrl_node():
    def __init__(self):
        rospy.Subscriber('/ardrone/odometry_sensor1/odometry',Odometry,self.navdata_callback)
        rospy.Subscriber('/ardrone/ground_truth/odometry',Odometry,self.vrpn_callback)
        rospy.Subscriber('ardrone/camera_nadir/image_raw/compressed',CompressedImage,self.image_callback,queue_size = 1)
        self.pubcmd = rospy.Publisher('/ardrone/command/roll_pitch_yawrate_thrust',RollPitchYawrateThrust)
        self.position=[0]*3
        self.velocity=[0]*3
        self.rpy=[0]*3
        self.cmd=[0]*4
        self.sim_time=0
        self.mpc = Mpc()
        self.mpc_time_last_u = 0
        self.mpc_time_last_c = 0

    def vrpn_callback(self,data):
        self.sim_time = data.header.stamp.secs + data.header.stamp.nsecs * 0.000000001
        self.position[0] = data.pose.pose.position.x
        self.position[1] = data.pose.pose.position.y
        self.position[2] = data.pose.pose.position.z
        self.velocity[0] = data.twist.twist.linear.x
        self.velocity[1] = data.twist.twist.linear.y
        self.velocity[2] = data.twist.twist.linear.z
        self.mpc_ctrl()
        self.sendCommand()
        #print self.sim_time,"%.3f,%.3f,%.3f,%.3f"%(self.cmd[0],self.cmd[1],self.cmd[2],self.cmd[3])

    def image_callback(self,ros_data):
    	np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        detect_markers(image_np)
        cv2.imshow('cv_img', image_np)
        cv2.waitKey(2)

    def navdata_callback(self,data):
        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.rpy[0] = euler[0]
        self.rpy[1] = euler[1]
        self.rpy[2] = euler[2]

    def mpc_ctrl(self):
        if self.mpc_time_last_u == 0 or self.sim_time - self.mpc_time_last_u >=0.1:
            self.mpc_u = self.mpc.calc_U(self.position[0],self.position[1],self.position[2],
                self.velocity[0],self.velocity[1],self.velocity[2],self.sim_time)
        if self.mpc_time_last_c == 0 or self.sim_time - self.mpc_time_last_c >=0.01:
            self.mpc_c = self.mpc.calc_ctrl(self.rpy[0],self.rpy[1],self.rpy[2])
        self.cmd[0] = self.mpc_c[0]
        self.cmd[1] = self.mpc_c[1]
        self.cmd[2] = self.mpc_c[2]
        self.cmd[3] = self.mpc_c[3]

    def pid_position_ctrl(self,x,y,z):
        self.cmd[3] = 1.52 * 9.8 + 0.5 * (z - self.pose[2])

    def sendCommand(self):
        msg = RollPitchYawrateThrust()
        msg.roll = self.cmd[0]
        msg.pitch = self.cmd[1]
        msg.yaw_rate = self.cmd[2]
        msg.thrust.z = self.cmd[3]
        self.pubcmd.publish(msg)

def onClose():
    print "Closing..."

if __name__ == '__main__':
    rospy.init_node('ardrone_ctrl', anonymous=True)
    rospy.on_shutdown(onClose)
    acn = ardrone_ctrl_node()
    rospy.spin()
