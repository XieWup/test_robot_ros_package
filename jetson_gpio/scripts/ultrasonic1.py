#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from serial import Serial
import serial.tools.list_ports
from sensor_msgs.msg import Range
from std_msgs.msg import Header
import struct

import math 
import os,sys
import time
import threading


x=serial.Serial('/dev/Ultrasound',9600)
def faSong():   # 发送函数
    while True: # 循环发送数据
          
        myinput=b'\xb8\x01'    # 需要发送的十六进制数据
        x.write(myinput)    # 用write函数向串口发送数据
        time.sleep(0.5)      # 设置发送间隔时间
        #print(x)
def jieShou():  # 接收函数
    while True: # 循环接收数据
        while x.inWaiting()>0:  # 当接收缓冲区中的数据不为零时，执行下面的代码
            myout=x.read(24) # 提取接收缓冲区中的前7个字节数
            #print (myout)
            datas=''.join(map(lambda x:('/x' if len(hex(x))>=4 else '/x0')+hex(x)[2:],myout))
            #print(datas)
            new_datas=datas[2:].split('/x') # 由于datas变量中的数据前两个是/x，所以用到切片工具
            need=''.join(new_datas) # 将前面的列表中的数据连接起来
           
            print(need)
            data_1=float(need[4]+need[5]+need[6]+need[7])/1000
            data_2=float(need[16]+need[17]+need[18]+need[19])/1000
            data_3=float(need[28]+need[29]+need[30]+need[31])/1000
            #data_4=float(need[40]+need[41]+need[42]+need[43])/1000
           
            print(data_1,data_2,data_3)
            msg1=Range()
            msg2=Range()
            msg3=Range()
            #msg4=Range()

            header1=Header()
            header2=Header()
            header3=Header()
            header4=Header()

            header1.stamp=rospy.Time.now()
            header2.stamp=rospy.Time.now()
            header3.stamp=rospy.Time.now()
            #header4.stamp=rospy.Time.now()

            header1.frame_id="ultrasound1"
            header2.frame_id="ultrasound2"
            header3.frame_id="ultrasound3"
           # header4.frame_id="ultrasound4"

            msg1.header=header1
            msg2.header=header2
            msg3.header=header3
            #msg4.header=header4

            msg1.field_of_view=1
            msg2.field_of_view=1
            msg3.field_of_view=1
            #msg4.field_of_view=1

            msg1.min_range=0.25
            msg2.min_range=0.25
            msg3.min_range=0.25
            #msg4.min_range=0.25

            msg1.max_range=0.5 #1.5
            msg2.max_range=0.5 #1.5
            msg3.max_range=0.5 #1.5
            #msg4.max_range=0.5 #1.5

            msg1.range=data_1
            msg2.range=data_2
            msg3.range=data_3
           # msg4.range=data_4

            global cmd_pub1,cmd_pub2,cmd_pub3  #cmd_pub4
            cmd_pub1.publish(msg1)
            cmd_pub2.publish(msg2)
            cmd_pub3.publish(msg3)
            #cmd_pub4.publish(msg4)
if __name__ == '__main__':
    rospy.init_node('ultrasonic')
    global cmd_pub1,cmd_pub2,cmd_pub3 #cmd_pub4
    cmd_pub1 = rospy.Publisher('/sonar1',Range,queue_size=10)
    cmd_pub2 = rospy.Publisher('/sonar2',Range,queue_size=10)
    cmd_pub3 = rospy.Publisher('/sonar3',Range,queue_size=10)
    #cmd_pub4 = rospy.Publisher('/sonar4',Range,queue_size=10)
    t1=threading.Thread(target=jieShou)
    t2=threading.Thread(target=faSong)
    t2.start()
    t1.start()
