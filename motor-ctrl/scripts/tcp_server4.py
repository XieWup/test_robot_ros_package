#!/usr/bin/env python
# -*- coding: utf-8 -*-
from socket import *
from time import ctime
import time

import rospy
import sys
from std_msgs.msg import Bool
import actionlib  
from actionlib_msgs.msg import *  
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import BatteryState

HOST = '192.168.1.105'
PORT = 6000 #端口号
BUFSIZ = 1024 #接收数据缓冲大小
ADDR = (HOST, PORT)

print('本机作为服务端')
print('本机IP：')
print(HOST)
print('端口：')
print(PORT)
tcpSerSock = socket(AF_INET, SOCK_STREAM) #创建TCP服务器套接字
tcpSerSock.bind(ADDR) #套接字与地址绑定
tcpSerSock.listen(8) #监听连接，同时连接请求的最大数目


print('等待客户机的连接')
tcpCliSock, addr = tcpSerSock.accept()  #接收继电器端连接请求
print('连接成功')
print('客户端IP与端口如下:')
print(addr)

class tcp_server:
  def __init__(self):
    self.btn_state = False
    self.sub = rospy.Subscriber("btn_press", Bool, self.callback)
    pass
  def read_DI(self):
    print('获取开关量输入接口1的状态发送：AT+OCCH1=?\\r\\n')
    self.D1 = "AT+OCCH1=?\r\n"
    self.tcpCliSock.send(self.D1.encode())
    self.recv_data = self.tcpCliSock.recv(BUFSIZ)
    self.DI1 = (recv_data.split(':')[1])
    if DI1==0 and 
      
    print('网络继电器应答：')
    print(self.recv_data.decode('gbk'))
    print('指令执行成功!')

  def callback(self,data):   
    self.btn_state = True
    self.target = Pose(Point(0.000, 0.001, 0.000), Quaternion(0.000, 0.000, 0.013,0.999))
    #呼叫电梯点
    self.goal = MoveBaseGoal()  
    self.goal.target_pose.pose = self.target
    self.goal.target_pose.header.frame_id = 'map'  
    self.goal.target_pose.header.stamp = rospy.Time.now() 
    rospy.loginfo("Going to: " + str(self.target)) 
    # 向目标进发  
    self.move_base.send_goal(self.goal)  
    # 五分钟时间限制  
    self.finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))   
    # 查看是否成功到达  
    if not self.finished_within_time:  
        self.move_base.cancel_goal()  
        rospy.loginfo("Timed out achieving goal")  
    else:  
        state = self.move_base.get_state()  
        if state == GoalStatus.SUCCEEDED:  
            rospy.loginfo("Goal succeeded!")
            self.DO1 = "AT+STACH1=1\r\n"
            self.tcpCliSock.send(self.DO1.encode())
            self.recv_data = tcpCliSock.recv(BUFSIZ)
            #print("self.staue is :" + self.staue)
        else:  
            rospy.loginfo("Goal failed！ ")


while not rospy.is_shutdown():
  print('TCP-KP系列网络继电器功能测试：')
  #================================================
  print('获取开关量输入接口1的状态发送：AT+OCCH1=?\\r\\n')
  meg = "AT+OCCH1=?\r\n"
  tcpCliSock.send(meg.encode())
  recv_data = tcpCliSock.recv(BUFSIZ)
  print('网络继电器应答：')
  #print(recv_data.replace("\r\n", "\\r\\n").decode('gbk'))
  print(recv_data.decode('gbk'))
  print('指令执行成功!')
  time.sleep(0.5)#休眠0.5秒

  print('控制继电器通道1常开吸合发送：AT+STACH1=1\\r\\n')
  meg = "AT+STACH1=1\n"
  tcpCliSock.send(meg.encode())
  recv_data = tcpCliSock.recv(BUFSIZ)
  print('网络继电器应答：')
  print(recv_data.decode('gbk'))
  print('指令执行成功!')
  time.sleep(0.5)#休眠0.5秒

#================================================
  print('控制继电器通道1常开断开发送：AT+STACH1=0\\r\\n')
  meg = "AT+STACH1=0\r\n"
  tcpCliSock.send(meg.encode())
  recv_data = tcpCliSock.recv(BUFSIZ)
  print('网络继电器应答：')
  print(recv_data.decode('gbk'))
  print('指令执行成功!')
  time.sleep(0.5)#休眠0.5秒


  
#print('关闭客户端连接！')
#tcpCliSock.close() #关闭与继电器的连接
#tcpSerSock.close() #关闭服务器socket
#print('测试完成！')


