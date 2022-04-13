#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import numpy as np
from std_msgs.msg import Float64
import geometry_msgs.msg
import tf2_ros

# this    is    for   topic   publisher
odom_pub = rospy.Publisher('/cart_odom', Odometry, queue_size = 10)
imu_pub = rospy.Publisher('/imu/data', Imu, queue_size = 10)

odom = Odometry()
imu  =  Imu()

def Quaternion2RPY(qx, qy, qz, qw) :
    r11 = 1.0 - 2.0*qy**2 - 2.0*qz**2
    r12 = 2.0*qx*qy - 2.0*qz*qw
    r13 = 2.0*qx*qz + 2.0*qy*qw
    r21 = 2.0*qx*qy + 2.0*qz*qw
    r22 = 1.0 - 2.0*qx**2 - 2.0*qz**2
    r23 = 2.0*qy*qz - 2.0*qx*qw
    r31 = 2.0*qx*qz - 2.0*qy*qw
    r32 = 2.0*qy*qz +2.0*qx*qw
    r33 = 1.0 - 2.0*qx**2 - 2.0*qy**2

    rot2 = np.array([[r11,r12,r13], [r21,r22,r23], [r31,r32,r33]])
    yaw = np.arctan2(r21,r11)
    pitch = np.arcsin(-r31)
    roll = np.arctan2(r32,r33)

    return roll, pitch, yaw


def callback(data):
    global odom
    global odom_pub
    global imu_pub
    global imu

    pos_x = data.pose.pose.position.x
    pos_y = data.pose.pose.position.y
    pos_z = data.pose.pose.position.z

    qx = data.pose.pose.orientation.x
    qy = data.pose.pose.orientation.y
    qz = data.pose.pose.orientation.z
    qw = data.pose.pose.orientation.w

    vel_x = data.twist.twist.linear.x
    vel_y = data.twist.twist.linear.y
    vel_z = data.twist.twist.linear.z

    ang_vel_x = data.twist.twist.angular.x
    ang_vel_y = data.twist.twist.angular.y
    ang_vel_z = data.twist.twist.angular.z

    r,p,yaw = Quaternion2RPY(qx, qy, qz, qw)

    sensor_offset = 0.11 #0.365 #0.22323 #0.194 #0.37   #0.365 

    pos_x = pos_x + sensor_offset

    x_off  = sensor_offset*np.cos(yaw)
    y_off = sensor_offset*np.sin(yaw)

    xu = pos_x - (x_off)
    yu = pos_y - (y_off)

    WR = ang_vel_z*sensor_offset

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.frame_id = "cart_odom_frame"
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = "base_link"
    t.transform.translation.x = xu
    t.transform.translation.y = yu
    t.transform.translation.z = pos_z

    t.transform.rotation.x = qx
    t.transform.rotation.y = qy
    t.transform.rotation.z = qz
    t.transform.rotation.w = qw
    br.sendTransform(t)

    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = "cart_odom_frame"

    odom.pose.pose.position.x = xu
    odom.pose.pose.position.y = yu
    odom.pose.pose.position.z = pos_z
    
    odom.pose.pose.orientation.x = qx
    odom.pose.pose.orientation.y = qy
    odom.pose.pose.orientation.z = qz
    odom.pose.pose.orientation.w = qw

    odom.child_frame_id = "base_link"

    odom.twist.twist.linear.x = vel_x
    odom.twist.twist.linear.y = 0.0

    odom.twist.twist.angular.z = ang_vel_z

    odom_pub.publish(odom)

    imu.header.stamp = rospy.Time.now()
    imu.header.frame_id ="imu_frame"
    imu.orientation.x = qx
    imu.orientation.y = qy
    imu.orientation.z = qz
    imu.orientation.w = qw

    imu_pub.publish(imu)

def listener():
    rospy.init_node("listener", anonymous = True)

    #rospy.Subscriber("/camera/odom/sample", Odometry, callback)
    rospy.Subscriber("/t265/odom/sample", Odometry, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
























