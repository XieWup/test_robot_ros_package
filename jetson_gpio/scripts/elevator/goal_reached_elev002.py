#!/usr/bin/env python 
# -*- coding: utf-8 -*-

import rospy
from actionlib_msgs.msg import *  
import actionlib
from geometry_msgs.msg import PoseWithCovarianceStamped,  PoseStamped, Pose, PoseArray, Twist
from move_base_msgs.msg import  MoveBaseActionResult, MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
from rospy.core import NullHandler  
from std_msgs.msg import Int64
import os
import sys
import time
import threading

class Goal_reached :

    def __init__(self):
        self.listener()
        

    def listener(self):
        rospy.init_node('goal_reached_elev', anonymous=True)
        #self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        #self.move_base.wait_for_server(rospy.Duration(60))  
        rospy.loginfo("goal_reached_elev004 start")  
        
        #self.call_elevator_sub = rospy.Subscriber('/number_floors',Int64,self.elev_callback1) #订阅要去往几楼
        self.out_elevator_sub = rospy.Subscriber('/special_points',Int64,self.elev_callback4)   
        self.arrived_elev_pub = rospy.Publisher('/arrived_elev',Int64,queue_size=10)   #特征点发布的话题 内容为 date:1  date:2 date:3
        #self.arrived_elev_sub = rospy.Subscriber('/arrived_elev',Int64, self.elev_callback2)
        self.current_pose_sub= rospy.Subscriber("/tracked_pose", PoseStamped,self.current_callback)
        self.result_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.result_callback)
        self.goal_sub = rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, self.goal_callback3)
        self.pub_simple_goal = rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=10)   #发布电梯点用
        self.pub_finish_ele = rospy.Publisher('/finish_ele',Int64,queue_size=10)   #电梯通讯完成后

        #self.pub_to_RawSim = 

        self.pub_cmd_vel = rospy.Publisher('/cmd_vel',Twist,queue_size=100)  #电梯通讯完成后切换地图

        self.test_switch_map = rospy.Publisher('/switch_map',Int64,queue_size=10)  #电梯通讯完成后切换地图

        self.call_elev =0
        self.getstatusList=list()
        #self.arrived_pose=0
        self.special_status = 0
        #self.__number_status = False #设置接收电梯点的初始状态
        self.data1=Int64()
        
        #self.in_ele_point = MoveBaseGoal()    #电梯内的点位
        #self.in_ele_point.target_pose.pose.position.x = -0.8255543776220731
        #self.in_ele_point.target_pose.pose.position.y = -0.11118848660122817
        #self.in_ele_point.target_pose.pose.orientation.z = -0.7214382707978507
        #self.in_ele_point.target_pose.pose.orientation.w = 0.6924787516077351
        
        #self.out_ele_point = MoveBaseGoal()  #呼叫电梯/出电梯点
        #self.out_ele_point.target_pose.pose.position.x = -0.8832419046382717
        #self.out_ele_point.target_pose.pose.position.y = -1.612995809735922
        #self.out_ele_point.target_pose.pose.orientation.z = -0.9998462295852648
        #self.out_ele_point.target_pose.pose.orientation.w = -0.017536167886112675

        self.elepoint1 = PoseStamped() #map1电梯外的点
        self.elepoint1.header.stamp = rospy.Time.now()
        self.elepoint1.header.frame_id = "map"
        self.elepoint1.pose.position.x = -0.9583786258898745
        self.elepoint1.pose.position.y = 0.07979238081784144
        self.elepoint1.pose.orientation.z = -0.7079645366858975
        self.elepoint1.pose.orientation.w = 0.706247983922873

        self.elepoint2 = PoseStamped() #map1电梯内的点
        self.elepoint2.header.stamp = rospy.Time.now()
        self.elepoint2.header.frame_id = "map"
        self.elepoint2.pose.position.x = -0.9683662517964299
        self.elepoint2.pose.position.y = -1.4634543679317693
        self.elepoint2.pose.orientation.z = -0.7132739553310873
        self.elepoint2.pose.orientation.w = 0.700885343438102

        self.elepoint3 = PoseStamped() #map2电梯内的点
        self.elepoint3.header.stamp = rospy.Time.now()
        self.elepoint3.header.frame_id = "map"
        self.elepoint3.pose.position.x = 0.004946822689451083
        self.elepoint3.pose.position.y = 0.00401618407475496
        self.elepoint3.pose.orientation.z = 0.005363214265013155
        self.elepoint3.pose.orientation.w = 0.9999856178629508

        self.elepoint4 = PoseStamped() #map2电梯外的点
        self.elepoint4.header.stamp = rospy.Time.now()
        self.elepoint4.header.frame_id = "map"
        self.elepoint4.pose.position.x = -1.37540386619
        self.elepoint4.pose.position.y = -0.00231534009357
        self.elepoint4.pose.orientation.z = 0.0224812848982
        self.elepoint4.pose.orientation.w = 0.999747263977

        self.elepoint5 = PoseStamped() #map2电梯外的点,掉转方向，面朝工厂
        self.elepoint5.header.stamp = rospy.Time.now()
        self.elepoint5.header.frame_id = "map"
        self.elepoint5.pose.position.x = -1.7240807769568327
        self.elepoint5.pose.position.y = -0.001107166742904779
        self.elepoint5.pose.orientation.z = -0.9999541520667858
        self.elepoint5.pose.orientation.w = 0.00957568610571898

        self.rotation = Twist()
        self.rotation.angular.z = 0.5
   
        #self.goal_lists=[self.in_ele_point, self.out_ele_point]
        #t1 = threading.Thread(target=self.thread_spin)  # 末端位置订阅线程
        #t1.start()
        t2 = threading.Thread(target=self.thread_wait_for_message)
        t2.start()

    def thread_wait_for_message(self):
        pose0 = rospy.wait_for_message('/number_floors', Int64, timeout=None)
        self.call_elev =0
        self.numfloor = int(str(pose0)[-1:])
        if self.numfloor == 2 or self.numfloor == 3:
            print("发送地图1的电梯点1")
            self.goal = self.elepoint1
            self.pub_simple_goal.publish(self.goal)
        else :
            print("发送地图2的电梯点1")
            self.goal = self.elepoint4
            self.pub_simple_goal.publish(self.goal)
        X = self.goal.pose.position.x
        Y = self.goal.pose.position.y
        print(str(X)+str(Y))
        print("电梯点1发送完毕")
        while self.call_elev != 1 :
            print("等待机器人走向电梯点1: " + str(self.call_elev))
            time.sleep(1)
                #self.move_base.send_goal( self.goal_lists[1])  #发送呼叫电梯点
                #if self.elevator_X-0.2 < self.current_pose_x <= self.elevator_X +0.2  and  self.elevator_Y-0.2 < self.current_pose_y <= self.elevator_X+0.2:
                    #print("已进入IF内部")
                #if self.call_elev ==1: #当机器人当前位置在目标位置范围内 AND status=3 发布到达点位状态信息
            #print(self.arrived_pose)
        print("arrived P1")
        self.call_elev =0
            #self.data1=Int64()
        #self.data1.data=1
        #self.arrived_elev_pub.publish(self.data1)
        self.elev_callback2()
            #self.call_elev =0
            #self.arrived_pose=0
            #self.__number_status = False

        
        
    def elev_callback1(self,pose0):  #逻辑：当订阅number_floors话题消息不为空时，将呼叫电梯点通过move_base服务器传递给move_base
        #if pose0 != NullHandler:
            #self.__number_status = True
        self.call_elev =0
        self.numfloor = int(str(pose0)[-1:])
        if self.numfloor == 2 or self.numfloor == 3:
            print("发送地图1的电梯点1")
            self.goal = self.elepoint1
            self.pub_simple_goal.publish(self.goal)
        else :
            print("发送地图2的电梯点1")
            self.goal = self.elepoint4
            self.pub_simple_goal.publish(self.goal)
        X = self.goal.pose.position.x
        Y = self.goal.pose.position.y
        print(str(X)+str(Y))
        print("电梯点1发送完毕")
        while self.call_elev != 1 :
            print("等待机器人走向电梯点1: " + str(self.call_elev))
            time.sleep(1)
                #self.move_base.send_goal( self.goal_lists[1])  #发送呼叫电梯点
                #if self.elevator_X-0.2 < self.current_pose_x <= self.elevator_X +0.2  and  self.elevator_Y-0.2 < self.current_pose_y <= self.elevator_X+0.2:
                    #print("已进入IF内部")
                #if self.call_elev ==1: #当机器人当前位置在目标位置范围内 AND status=3 发布到达点位状态信息
            #print(self.arrived_pose)
        print("arrived P1")
        self.call_elev =0
            #self.data1=Int64()
        #self.data1.data=1
        #self.arrived_elev_pub.publish(self.data1)
        self.elev_callback2()
            #self.call_elev =0
            #self.arrived_pose=0
            #self.__number_status = False

             
    def elev_callback2(self):
        self.call_elev =0
        #self.__number_status = True
        print("进入elev_callback2")
        #print(msg1)
        #if str(msg1) == "data: 1":
            #print(self.special_status)
        while self.special_status != 2:
            print("等待电梯门打开: " + str(self.special_status))
            time.sleep(1)
                #if self.special_status ==2:
        if self.numfloor == 2 or self.numfloor == 3:
            self.goal = self.elepoint2
            self.pub_simple_goal.publish(self.goal)
        else :
            self.goal = self.elepoint3
            self.pub_simple_goal.publish(self.goal)
        X = self.goal.pose.position.x
        Y = self.goal.pose.position.y
        print(str(X)+str(Y))
        print("电梯点2发送完毕")
                #self.move_base.send_goal( self.goal_lists[0])

                    #if self.elevator_X-0.2 < self.current_pose_x <=self.elevator_X +0.2  and  self.elevator_Y-0.2 < self.current_pose_y <= self.elevator_Y+0.2 :
        while self.call_elev != 1 :
            print("等待机器人走入电梯内...")
            time.sleep(1)
            #print(self.arrived_pose)
        print("arrived P2")
        self.call_elev =0
        if self.numfloor == 2 or self.numfloor == 3:
            self.test_switch_map.publish(self.numfloor)  #切换成2or3地图
            #self.data1=Int64()
        #self.data1.data=2
        #self.arrived_elev_pub.publish(self.data1)
            #self.call_elev =0
            #self.arrived_pose=0
            #self.__number_status = False
                #else : 
                    #print("等待电梯门打开: " + str(self.special_status))
                    #time.sleep(1)
        #elif str(msg1) == "data: 2":
        while self.special_status != 3:
            print("等待电梯门打开: " + str(self.special_status))
            time.sleep(1)
                #if self.special_status ==3:
        if self.numfloor == 2 or self.numfloor == 3:
            self.goal = self.elepoint4
            self.pub_simple_goal.publish(self.goal)#可以试试elepoint5
        else :
            self.goal = self.elepoint4
            self.pub_simple_goal.publish(self.goal)# 待上面尝试之后 ，如果可行，再加一个面朝工厂的map1的电梯外的点 
            #self.pub_simple_goal.publish(self.elepoint1) #优化方向：将发布点位写在while循环之外
        X = self.goal.pose.position.x
        Y = self.goal.pose.position.y
        print(str(X)+str(Y))
        print("电梯点1发送完毕")
                #self.move_base.send_goal( self.goal_lists[1])
                    #if self.elevator_X-0.2 < self.current_pose_x <=self.elevator_X +0.2  and  self.elevator_Y-0.2 < self.current_pose_y <= self.elevator_Y+0.2 :
                        #if self.call_elev ==1:
        while self.call_elev != 1 :
            print("等待机器人走出电梯...")
            time.sleep(1)
            #print(self.arrived_pose)
        print("arrived P3")
                            #self.data1=Int64()
        self.call_elev =0
            #self.data1.data=3
            #print("self.numfloor:" + str(self.numfloor))
        if self.numfloor == 1:
            self.test_switch_map.publish(self.numfloor) #切换地图
            time.sleep(3)
                #旋转90度
            for i in range(32):
                #计量关系：
                # 360度=2*pi弧度 即180度 = 3.1415927
                # self.rotation.angular.z = 0.5: 0.5弧度/s
                # time.sleep(0.1) 循环63次 = 6.3s
                # 0.5 * 3.2 = 1.6            
                self.pub_cmd_vel.publish(self.rotation)
                time.sleep(0.1) #与queue_size频率相同  看起来最顺
        elif self.numfloor == 2 or self.numfloor == 3:
                #旋转180度
            for i in range(64):
                #计量关系：
                # 360度=2*pi弧度 即180度 = 3.1415927
                # self.rotation.angular.z = 0.5: 0.5弧度/s
                # time.sleep(0.1) 循环63次 = 6.3s
                # 0.5 * 6.3 = 3.15            
                self.pub_cmd_vel.publish(self.rotation)
                time.sleep(0.1) #与queue_size频率相同  看起来最顺
            #self.arrived_elev_pub.publish(self.data1)    明月写的，但这里导致多次出发回调函数
        self.pub_finish_ele.publish(1)
        self.special_status = 0
        #os.system('cd /home/hanning/robot_ws/src/jetson_gpio/scripts && ./goal_reached_elev004.sh')
        #else : print("")

            

            #self.arrived_pose=0
            #self.__number_status = False
                            #break
    def goal_callback3(self,pose1):
        self.elevator_X = pose1.goal.target_pose.pose.position.x
        self.elevator_Y = pose1.goal.target_pose.pose.position.y

    def elev_callback4(self,msg2):
        self.special_status = msg2.data
        
            
    def current_callback(self,Pose):  #获取机器人当前位置的x y坐标
        self.current_pose_x = Pose.pose.position.x
        self.current_pose_y = Pose.pose.position.y
        #print(self.current_pose_x)
            
    def result_callback(self,msg):
        if msg.status.status ==3:   
            self.call_elev =1
        print(msg.status.status)
        print("self.call_elev = "+str(self.call_elev))

    def thread_spin(self):
        rospy.spin()

#def thread_spin_all():
    #rospy.spin()
            
            

if __name__ == '__main__':  
    try:  
        Goal_reached()  
        rospy.spin()
        #input("请输入任意按键退出")

    except rospy.ROSInterruptException:  
        rospy.loginfo("Exploring SLAM finished.")
