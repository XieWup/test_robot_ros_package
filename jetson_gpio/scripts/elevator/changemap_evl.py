#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import cmd
import rospy
import roslib
import actionlib
import time
from actionlib_msgs.msg import *
from rospy.core import NullHandler
from std_msgs.msg import String,Int64
from jetson_gpio.msg import data
from move_base_msgs.msg import MoveBaseAction,  MoveBaseActionResult
from geometry_msgs.msg import PoseStamped
import threading


class ChangeMap :
    
    def __init__(self):
        rospy.init_node('changemap_evl', anonymous=True)
        #self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        #rospy.loginfo("Waiting for move_base action server...")
        #self.move_base.wait_for_server(rospy.Duration(60))
        #rospy.loginfo("Connected to move base server")
        self.map_id_sub = rospy.Subscriber('/switch_map', Int64, self.callback1)       #  pub /map_id std_msgs/Int64 "data: 1"
	#rospy.slepp(1)
        self.IO_sub = rospy.Subscriber('/IO',data, self.callback2)
        self.elevator_point_sub = rospy.Subscriber('/elevator_point', PoseStamped, self.callback4)
        self.result_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.result_callback)
        rospy.Subscriber('wait_build_signal', Int64, self.stringSubscriberCallback) 
        rospy.Subscriber('end_build_signal', Int64, self.endSubscriberCallback) 
        self.elevator_point_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped,queue_size=10)
        self.elevator_pub = rospy.Publisher("/map_id", Int64, queue_size=10)
        self.map_id_pub=rospy.Publisher("/map_reload", String, queue_size=10)
        self.ID=1
        self.M=0
        self.call_elevator = 0
	

        
    def callback1(self,msg):
        self.num = str(msg)
        self.map_id= "map"+self.num[-1:]
        self.loc_map_id= msg.data
	print("***********************************************************************************************************************************************")
        print(self.loc_map_id)
        
        if self.num[-1:] == str(1):
            print(self.map_id)
              
            self.map_id_pub.publish(self.map_id)
            
            self.ID=1
            #self.local_map(ID)
            #os.system('cd /home/hanning/robot_ws/src/jetson_gpio/scripts/elevator  && ./map1.sh ')
	    os.system('gnome-terminal -x bash -c "roslaunch hanning_navigation localizatiom_map1.launch"')

            # add at 2022.5.11 by xie for pub map_id
            rate = rospy.Rate(10) # 10hz
            while not rospy.is_shutdown():
                self.elevator_pub.publish(1)
                rate.sleep()
            
        elif self.num[-1:] ==str(2):
            self.map_id_pub.publish(self.map_id)
           
            self.ID=2
            #self.local_map(ID)
            #os.system('cd /home/hanning/robot_ws/src/jetson_gpio/scripts/elevator  && ./map2.sh ')
	    os.system('gnome-terminal -x bash -c "roslaunch hanning_navigation localizatiom_map2.launch"')
            
        elif self.num[-1:] ==str(3):
            self.map_id_pub.publish(self.map_id)
           
            self.ID=3
            #self.local_map(ID)
            #os.system('cd /home/hanning/robot_ws/src/jetson_gpio/scripts/elevator  && ./map3.sh ')
	    os.system('gnome-terminal -x bash -c "roslaunch hanning_navigation localizatiom_map3.launch"')

    def callback2(self,data):
        self.elevator_pub.publish(self.ID)
                      
            
    def callback4(self,Pose):
        if Pose != NullHandler :
            self.elevator_point = Pose
            self.elevator_point_pub.publish(self.elevator_point)
            #print (self.elevator_point)
            
    def result_callback(self,msgs):
        if msgs.status.status ==3:
            self.call_elevator = 1
            #print(self.call_elevator)

    def stringSubscriberCallback(self,data): 
        rospy.loginfo('start building map...')
        os.system('gnome-terminal -x bash -c "roslaunch hanning_navigation build_map.launch"')
        os.system('gnome-terminal -x bash -c "rosrun rosOpenCV sub_map.py"')

    def endSubscriberCallback(self,data):
        map_id = Int64()
        map_id = data.data
        rospy.loginfo('end building map and save it...')
        #save map
        os.system('rosservice call /finish_trajectory 0 ')
        time.sleep(3)
        os.system('rosservice call /write_state "{filename: \'${HOME}/robot_ws/src/motor-ctrl/mymaps/map'+str(map_id)+'.pbstream\', include_unfinished_submaps: "true"}"')
        time.sleep(3)
        os.system('rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=${HOME}/robot_ws/src/motor-ctrl/mymaps/map'+str(map_id)+' -pbstream_filename=${HOME}/robot_ws/src/motor-ctrl/mymaps/map'+str(map_id)+'.pbstream -resolution=0.03 ')
        time.sleep(3)
        os.system('sed -i \'1i map_id: map'+str(map_id)+'\' /home/hanning/robot_ws/src/motor-ctrl/mymaps/map'+str(map_id)+'.yaml')
        

            
if __name__ == '__main__':
        #try:  
        ChangeMap()  
        rospy.spin()  

        #except rospy.ROSInterruptException:  
            #rospy.loginfo("Exploring changemap finished.")
        
        
    
