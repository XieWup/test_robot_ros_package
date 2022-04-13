#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import serial.tools.list_ports
import struct
from binascii import unhexlify
import binascii
import crcmod
import threading
from sensor_msgs.msg import BatteryState

rel_speed = 0
speed_v = 0      #mm/s
speed_a = 0   #mrad/s 
    
 #  ROS param  set

#linear_coef = rospy.get_param('~linear_coef ', default =1000 ) 
#angular_coef = rospy.get_param('~angular_coef', default = 1000)
#maxspeed_v = rospy.get_param('~maxspeed_v', default = 1000)
#maxspeed_a = rospy.get_param('~maxspeed_a', default = 1000)
linear_coef =1000
angular_coef =1000
maxspeed_v =1000
maxspeed_a =1000

motor_speed_mode = b'\x01\x2F\x60\x60\x00\x03\x00\x00\x00\x0D'
#motor_status = b'\x01\x43\x50\x00\x51\x00\x68\x95'
motor_start = b'\x01\x44\x21\x00\x31\x00\x00\x01\x00\x01\x75\x34'
motor_stop = b'\x01\x44\x21\x00\x31\x00\x00\x00\x00\x00\xE5\x34' 
Drop_current = b'\x01\x44\x24\x2A\x34\x2A\x01\xF4\x01\xF4\xF7\x43' 
Rising_current = b'\x01\x44\x24\x2A\x34\x2A\x08\x34\x08\x34\xF2\xE3'

class TrdDriver():

    def __init__(self, serialprot, baudrate):
        self.ser = serial.Serial(serialprot, baudrate)
        rospy.init_node('trd_driver_node')
        self.set_speed(speed_v, speed_a)
        self.vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.vel_callback)
        #self.vel_sub = rospy.Subscriber('/joy_vel', Twist, self.vel_callback1)
        self.vel_sub = rospy.Subscriber('/cmd_vel1', Twist, self.vel_callback2)
        self.vel_sub = rospy.Subscriber('battery',BatteryState,self.BatteryCallback)
        self.v2=0
        self.a2=0
        #self.ser.write(motor_start)
        #rospy.sleep(0.5)
        rospy.spin()
    def send(self, speed_code):
        self.ser.write(speed_code)

    def set_speed(self, speed_v, speed_a):
        byte0 = b'\x01'
        byte1 = b'\xEA'
      
        DEC1 = int(speed_v)
        DEC2 = int(speed_a)
        byte2 =(struct.pack("i",DEC1)[-4:-3])#线速度
        byte3 =(struct.pack("i",DEC1)[-3:-2])#线速度
#        speed_vel = byte2 + byte3
        byte4 = (struct.pack("i",DEC2)[-4:-3])    #角速度两个字节
        byte5 = (struct.pack("i",DEC2)[-3:-2])
#        speed_ang = byte4+byte5
        byte_speed = byte2 + byte3 + byte4 + byte5
        read = byte0 + byte1 + byte_speed  #解析校验码要输入的前几位
        read=(str(binascii.b2a_hex(read))[2:-1])
        crc16 =crcmod.mkCrcFun(0x18005, rev=True, initCrc=0xFFFF, xorOut=0x0000)
        data = read.replace(" ","")
    #print (data)
        readcrcout=hex(crc16(unhexlify(data))).upper()
        str_list = list(readcrcout)
        if len(str_list) < 6:
            str_list.insert(2, '0'*(6-len(str_list)))  # 位数不足补0
        crc_data = "".join(str_list)
#print(crc_data)
        read = read.strip()+crc_data[4:]+crc_data[2:4]
        read = bytes.fromhex(read)
        speed_code = read
#        print('speed_code', speed_code)
        self.send(speed_code)

    def vel_callback(self, msg):
        #print('test msg:')
        print('test msg:', msg.linear.x, msg.angular.z)
        v1 = (linear_coef * msg.linear.x )
        a1 = (angular_coef * msg.angular.z)
        #v1=(linear_coef * (v2+v3))
        #a1=(angular_coef *(a2+a3))
        print (v1,a1)

        speed_v = v1
        if speed_v >0:
           if speed_v <= maxspeed_v:
              speed_v = speed_v
           else:
              speed_v = maxspeed_v
              speed_v = speed_v
        else:
           if speed_v >= (-maxspeed_v):
              speed_v = speed_v
           else:
              speed_v =  (-maxspeed_v)
              speed_v = speed_v
#        speed_a = (self.speed_a) + a1
        speed_a = a1
        if speed_a >0:
           if speed_a <= maxspeed_a:
              speed_a = speed_a
           else:
              speed_a = maxspeed_a
              speed_a = speed_a
        else:
           if speed_a >= (-maxspeed_a):
               speed_a = speed_a
           else:
              speed_a =  (-maxspeed_a)
        
              speed_a = speed_a
        #print(speed_v,speed_a)

        self.set_speed(speed_v, speed_a)  


    def vel_callback2(self, cmd):
        #print(cmd)
        speed_v = (linear_coef * cmd.linear.x )
        speed_a = (angular_coef * cmd.angular.z)
        self.set_speed(speed_v, speed_a)

           
    def BatteryCallback(self, value):
        #print(value)
        battery = value.current
        if battery > 0:
            self.ser.write(Drop_current)
        else:
            self.ser.write(Rising_current)



if __name__ =='__main__':
    #serialprot = rospy.get_param('~serialprot', default='/dev/ttyS1')
    #baudrate = rospy.get_param('~baudrate', default=115200)
    serialprot = '/dev/ttyS0'
    baudrate = 115200
    m = TrdDriver(serialprot, baudrate)
    
