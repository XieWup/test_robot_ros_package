#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from serial import Serial
import serial.tools.list_ports
from sensor_msgs.msg import BatteryState
import time
import threading
import os,sys
from jetson_gpio.msg import data

x=serial.Serial('/dev/ttyUSB0',115200)

def faSong():   # 发送函数
    while True: # 循环发送数据
          
        myinput=b'\x01\x01\x00\x00\x00\x08\x3d\xcc'    # 需要发送的十六进制数据
        x.write(myinput)    # 用write函数向串口发送数据
        time.sleep(0.5)      # 设置发送间隔时间
    #print(myinput)
def jieShou():  # 接收函数
    while True: # 循环接收数据
        while x.inWaiting()>0:  # 当接收缓冲区中的数据不为零时，执行下面的代码
            myout=x.read(6) # 提取接收缓冲区中的前7个字节数
            recv=myout[3]
            if recv == 0:
              gpio = [0,0,0,0,0,0,0,0]
            elif recv == 1:
              gpio = [0,0,0,0,0,0,0,1]
            elif 1<recv<=3 :
              recv_number=dec2bin(recv)
              gpio = [recv_number[1],recv_number[0],0,0,0,0,0,0]
            elif 3<recv<=7 :
              recv_number=dec2bin(recv)
              gpio = [recv_number[2],recv_number[1],recv_number[0],0,0,0,0,0]
            elif 7<recv<=15 :
              recv_number=dec2bin(recv)
              gpio = [recv_number[3],recv_number[2],recv_number[1],recv_number[0],0,0,0,0]
            elif 15<recv<=31 :
              recv_number=dec2bin(recv)
              gpio = [recv_number[4],recv_number[3],recv_number[2],recv_number[1],recv_number[0],0,0,0]
            elif 31<recv<=63 :
              recv_number=dec2bin(recv)
              gpio = [recv_number[5],recv_number[4],recv_number[3],recv_number[2],recv_number[1],recv_number[0],0,0]
            elif 63<recv<=127 :
              recv_number=dec2bin(recv)
              gpio = [recv_number[6],recv_number[5],recv_number[4],recv_number[3],recv_number[2],recv_number[1],recv_number[0],0]
             
            elif 127<recv<=255 :
              recv_number=dec2bin(recv)
              gpio = [recv_number[7],recv_number[6],recv_number[5],recv_number[4],recv_number[3],recv_number[2],recv_number[1],recv_number[0]]

            
            recv_1=int(gpio[0])
            recv_2=int(gpio[1])
            recv_3=int(gpio[2])
            recv_4=int(gpio[3])
            recv_5=int(gpio[4])
            recv_6=int(gpio[5])
            recv_7=int(gpio[6])
            recv_8=int(gpio[7])
            print (recv_8)
            msg = data()
            msg.data1 = recv_1
            msg.data2 = recv_2
            msg.data3 = recv_3
            msg.data4 = recv_4
            msg.data5 = recv_5
            msg.data6 = recv_6
            msg.data7 = recv_7
            msg.data8 = recv_8
            global cmd_pub
            cmd_pub.publish(msg)
           




def dec2bin(num): #10jinzhi--2jinzhi
    mid =[]
    while True :
      num,rem = divmod(num,2)
      mid.append(str(rem))
      if num == 0:
        return ''.join(mid[::-1])
           
          
if __name__ == '__main__':
    rospy.init_node('batterypublisher1')
   # rospy.sleep(5)
    global cmd_pub
    cmd_pub = rospy.Publisher('IO',data,queue_size=1)
    t1=threading.Thread(target=jieShou)
    t2=threading.Thread(target=faSong)
    t2.start()
    t1.start()

