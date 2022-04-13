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

            
def listener():
    rospy.init_node('changemap_evl', anonymous=True)
    rospy.sleep(2)
    data = rospy.wait_for_message("/switch_map", Int64, timeout=None)
    num = str(data)
    #self.map_id= "map"+self.num[-1:]
    #self.loc_map_id= data.data
    print("***********************************************************************************************************************************************")
    print(num[-1:])
        
    if num[-1:] == str(1):
        #print(self.map_id)
              
        #self.map_id_pub.publish(self.map_id)
            
        #self.ID=1
            #self.local_map(ID)
            #os.system('cd /home/hanning/robot_ws/src/jetson_gpio/scripts/elevator  && ./map1.sh ')
        os.system('gnome-terminal -x bash -c "roslaunch hanning_navigation localizatiom_map1.launch"')
            
    elif num[-1:] ==str(2):
        #self.map_id_pub.publish(self.map_id)
           
        #self.ID=2
            #self.local_map(ID)
            #os.system('cd /home/hanning/robot_ws/src/jetson_gpio/scripts/elevator  && ./map2.sh ')
        os.system('gnome-terminal -x bash -c "roslaunch hanning_navigation localizatiom_map2.launch"')
            
    elif num[-1:] ==str(3):
        #self.map_id_pub.publish(self.map_id)
        #self.elevator_pub.publish(self.loc_map_id)
        #self.ID=3
        #self.local_map(ID)
        os.system('cd /home/hanning/robot_ws/src/jetson_gpio/scripts/elevator  && ./map3.sh ')

if __name__ == '__main__':
    listener()
