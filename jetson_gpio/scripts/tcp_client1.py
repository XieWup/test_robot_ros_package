#!/usr/bin/env python
# -*- coding: utf-8 -*-
from socket import *
from time import ctime
import time
import rospy 
import threading
from jetson_gpio.msg import data
import struct
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist

def main():
  #global cmd_pub,state
 # state=True
  datas=0
  
  while True:
    
      if datas==0:
        print('TCP-KP系列网络继电器功能测试：')
        meg1=b"\x00\x00\x00\x00\x00\x06\x01\x01\x00\x00\x00\x08"
        tcpCliSock.send(meg1.encode())
        recv_data1 = tcpCliSock.recv(BUFSIZ)
     # datas=''.join(map(lambda x:('/x' if len(hex(x))>=4 else '/x0')+hex(x)[2:],recv_data1))
    
        need=struct.unpack("b",recv_data1[9])
        
        recv_1=int(need[0])
       
        
        print(new_datas)
      #recv_data1=need.replace("\r", "\\r").decode('gbk')
      #recv_1=hex2dec(need)
      #if need=='\x00':
         
      #recv_1=recv_data1[8]
      #new_datas=recv_data1[8].split('/x')
     # new_datas=new_datas[2:].split('/x')
      #recv_1=int(need,16)
         

      if datas==0 and recv_1==0 :
         print('控制继电器通道1常开吸合发送：AT+STACH1=1\\r\\n')
         #BYTE Array[12] ={0x00,0x00,0x00,0x00,0x00,0x06,0x01,0x05,0x00,0x10,0x00,0x00}
        # send(tcpCliSock,(char*)Array,sizeof(Array),0)
         meg=b"\x00\x00\x00\x00\x00\x06\x01\x05\x00\x10\xff\x00"
         meg9 = b"\x00\x00\x00\x00\x00\x06\x01\x05\x00\x10\x00\x00"
         #meg=b"\x00\x00\x00\x00\x00\x08\x01\x0f\x00\x10\x00\x02\x01\x03"
         tcpCliSock.send(meg)
         recv_data11 = tcpCliSock.recv(BUFSIZ)
         time.sleep(0.5)
         tcpCliSock.send(meg9)
         recv_data9 = tcpCliSock.recv(BUFSIZ)
         time.sleep(0.5)
        # datas=1
         
      elif datas==1 :
         print('控制继电器通道1常开断开发送：AT+STACH1=1\\r\\n')
         meg9 = b"\x00\x00\x00\x00\x00\x06\x01\x05\x00\x10\x00\x00"
        # meg10 = "00 00 00 00 00 06 01 05 00 11 ff 00"
         tcpCliSock.send(meg9)
         recv_data9 = tcpCliSock.recv(BUFSIZ)
      #   time.sleep(0.5)
     #    tcpCliSock.send(meg10.encode())
         
     #    recv_data10 = tcpCliSock.recv(BUFSIZ)
         time.sleep(2)
         datas=2
      
#def gpio_node():

 #   rospy.Subscriber('battery',BatteryState,callback1)
 #   rospy.Subscriber('/smooth/cmd_vel', Twist, vel_callback)


if __name__ == '__main__':
    
    rospy.init_node('gpio')
  
    BUFSIZ = 1024 #接收数据缓冲大小
 
    tcpCliSock = socket(AF_INET, SOCK_STREAM) #创建TCP服务器套接字
 
    server_ip="192.168.1.200"
    server_port=4196
    tcpCliSock.connect((server_ip,server_port)) #客户机的连接

   # global cmd_pub
  #  cmd_pub = rospy.Publisher('tcp',data)
  #  gpio_node()
    main()
    rospy.spin()
 

