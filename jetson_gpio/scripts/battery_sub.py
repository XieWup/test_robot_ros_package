#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
#from serial import Serial
import serial
import serial.tools.list_ports
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float64
import time
import threading
import struct

x=serial.Serial('/dev/ttyS3',9600,timeout=0.5)
#x.open()
def faSong():   # 发送函数
    while True: # 循环发送数据
          
        myinput=b'\x7F\x10\x02\x06\x11\x58'    # 需要发送的十六进制数据  7F1002061158
        x.write(myinput)    # 用write函数向串口发送数据
        time.sleep(0.5)      # 设置发送间隔时间

def jieShou():  # 接收函数
    while True: # 循环接收数据
        if x.out_waiting > 0:  # 获取输出缓冲区的字节数
            myout=x.read(27) # 提取接收缓冲区中的前27个字节数
            '''
            struct模块可以解决bytes和其他二进制数据类型的转换
            struct的pack函数把任意数据类型变成bytes
            struct的unpack函数把bytes变成相应的数据类型
            struct.unpack('处理指令',要转换的数据)
            '''
            #intmyout = struct.unpack('<H',myout[21:23]) #tuple
            #print(type(myout[21:23]))  #bytes
            '''
            int.from_bytes()方法可以将字节值交换为int值
            要求：python3.2及以上
            '''
            soc = int.from_bytes(myout[21:23],"little") #剩余电量  
            electricity = int.from_bytes(myout[23:25],"little") #总电量
            result = round(soc / electricity,2) #电量的百分比
            print(result) #float
            global cmd_pub
            cmd_pub.publish(result)
            #print(electricity) #int
            #print(serial.to_bytes(myout[21:23])) #返回byte示例，在python2.x中，serial.read()是str，该功能可将str返回为byte

          
if __name__ == '__main__':
    rospy.init_node('batterypublisher1')
   # rospy.sleep(5)
    global cmd_pub
    cmd_pub = rospy.Publisher('battery',Float64,queue_size=10)
    t1=threading.Thread(target=jieShou)
    t2=threading.Thread(target=faSong)
    t2.start()
    t1.start()

