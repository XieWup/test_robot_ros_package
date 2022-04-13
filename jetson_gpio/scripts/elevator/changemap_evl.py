#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import cmd
import rospy
import roslib
import actionlib
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
            
        elif self.num[-1:] ==str(2):
            self.map_id_pub.publish(self.map_id)
           
            self.ID=2
            #self.local_map(ID)
            #os.system('cd /home/hanning/robot_ws/src/jetson_gpio/scripts/elevator  && ./map2.sh ')
	    os.system('gnome-terminal -x bash -c "roslaunch hanning_navigation localizatiom_map2.launch"')
            
        elif self.num[-1:] ==str(3):
            self.map_id_pub.publish(self.map_id)
            self.elevator_pub.publish(self.loc_map_id)
            self.ID=3
            self.local_map(ID)
            os.system('cd /home/hanning/robot_ws/src/jetson_gpio/scripts/elevator  && ./map3.sh ')

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
            
if __name__ == '__main__':
        #try:  
        ChangeMap()  
        rospy.spin()  

        #except rospy.ROSInterruptException:  
            #rospy.loginfo("Exploring changemap finished.")
        
        
    
