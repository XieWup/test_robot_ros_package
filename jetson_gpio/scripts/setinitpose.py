#!/usr/bin/env python

import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from random import sample
from math import pow, sqrt
import rospy
import subprocess
import signal

CONFIG_DIR = '/home/hanning/carto_ws/src/cartographer_ros/cartographer_ros/configuration_files'
CONFIG_BASE = 'sick_lidar.lua'

class SetInitialPose():
    def __init__(self):
        rospy.init_node('setinitpose', anonymous=True)
        
        rospy.on_shutdown(self.shutdown)
                
        # A variable to hold the initial pose of the robot to be set by 
        # the user in RViz        
        initial_pose = PoseWithCovarianceStamped()
        self.ready = False

        # Get the initial pose from the user
        rospy.loginfo("*** mClick the 2D Pose Estimate button in RViz to set the robot's initial pose...")
        
        rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)
        rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)

        while initial_pose.header.stamp == "":
            rospy.sleep(1)
            rospy.loginfo('.')
        
        print("Starting setinitpose" , initial_pose.pose.pose.position)                            
        while not rospy.is_shutdown():
            rospy.sleep(1)

            
    def update_initial_pose(self, msg):
        print('initial pos', msg.pose.pose.position)        
        print('initial orientation', msg.pose.pose.orientation)                
        self.initial_pose = msg
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        xp = msg.pose.pose.position.x
        yp = msg.pose.pose.position.y
        zp = msg.pose.pose.position.z

        s = 'rosservice call /finish_trajectory 1'
        print(s)
        process = subprocess.Popen(s, stdout=subprocess.PIPE, stderr=None, shell=True)
        output = process.communicate()
        print(output[0])
        
        s = 'rosservice call /start_trajectory "{configuration_directory: '
        s = s + '"' + CONFIG_DIR + '"'
        s = s + ', configuration_basename: "' + CONFIG_BASE + '", use_initial_pose: "true", initial_pose: '
        s = s + ' {position: {x: ' + str(xp) + ', y: ' + str(yp) + ', z: '+str(zp) + '}'
        s = s + ', orientation: {x: ' + str(x) + ', y: ' + str(y) + ', z: '+str(z)+ ', w: '+str(w)+'}}, relative_to_trajectory_id: 0}"'
        print(s)
        process = subprocess.Popen(s, stdout=subprocess.PIPE, stderr=None, shell=True)
        output = process.communicate()
        print(output[0])
        

    def shutdown(self):
        rospy.loginfo("Stopping setinitpose...")
        

if __name__ == '__main__':
    try:
        SetInitialPose()        
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("localize finished.")
