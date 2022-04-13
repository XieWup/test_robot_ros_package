#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import serial.tools.list_ports
import serial
import time
import math
#import threading
import struct
from binascii import unhexlify
#from crcmod import mkCrcFun
import binascii
import crcmod
#crcmod.pash.append("/usr/local/lib/python3.6/site-packages")

motor_speed_mode = b'\x01\x2F\x60\x60\x00\x03\x00\x00\x00\x0D'
#motor_status = b'\x01\x43\x50\x00\x51\x00\x68\x95'
motor_start = b'\x01\x44\x21\x00\x31\x00\x00\x01\x00\x01\x75\x34'
motor_stop = b'\x01\x44\x21\x00\x31\x00\x00\x00\x00\x00\xE5\x34'   # AB轴失能


def check_code(byte0, byte1, speed_vel, speed_ang):
        '计算校验码时需要输入的字节'
        read = byte0+byte1+speed_vel+speed_ang  #解析校验码要输入的前几位
        read=(str(binascii.b2a_hex(read))[2:-1])
        #print (read)
        return(read)

def crc16_modbus(read):
    '输出的控制电机指令字节码'
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
    #print(read)
    return (read)

def motor_speed_vel(speed_v):
    '计算小车线速度speed_v'
    DEC1 = (speed_v )
    byte2 =(struct.pack("i",DEC1)[-4:-3])#线速度
    byte3 =(struct.pack("i",DEC1)[-3:-2])#线速度
    speed_vel = byte2 + byte3
    return (speed_vel)
def motor_speed_ang(speed_a):
    '小车角速度speed_a'
    DEC2 = (speed_a)
    byte4 = (struct.pack("i",DEC2)[-4:-3])    #角速度两个字节
    byte5 = (struct.pack("i",DEC2)[-3:-2])
    speed_ang = byte4+byte5
    return (speed_ang)
#motor_speed_vel(200,20)

class SpeedMotor:
    def __init__(self, device, speed_v, speed_a):
        # 真实速度
        self.rel_speed = 0
        # 设置的速度
        self.set_speed1 = (speed_v)
        # 设置角速度
        self.set_speed2 = (speed_a)
        # 运行状态
        self.run = False
        # 故障状态
        self.fault = None
        # 电机电压
        self.voltage = 0
        # 电机电流
        self.current = 0
        # 设置串口通讯
        self.serial = serial.Serial(device, 115200)
        self.serial.timeout = 0
        # 设置为速度模式
        self.serial.write(motor_speed_mode)
        time.sleep(0.1)
        # 设置加减速度
 #       self.serial.write(b'\x0A\x14\x14\x32')
 #       time.sleep(0.1)

    def motor_speed_set(self):
        '速度设置'
        byte0 = b'\x01'
        byte1 = b'\xEA'
        speed_vel = motor_speed_vel(self.set_speed1)
        speed_ang = motor_speed_ang(self.set_speed2) 
        read = check_code(byte0, byte1, speed_vel, speed_ang)
        read = crc16_modbus(read)
        speed_code = read
        self.serial.write(speed_code)

    def motor_start(self):
        '启动'
        self.serial.write(motor_start)
        self.run = True
    def motor_stop(self):
        '停止'
        self.run = False
        self.serial.write(motor_stop)

#m = SpeedMotor('COM5', 0.4, 0.2)    #这里要改成nano对应的USB2口，如果插电脑测试就是COM3
m = SpeedMotor('/dev/ttyS0', 100, 20)
m.motor_start()
motor_speed_vel(100)
motor_speed_ang(20)
m.motor_speed_set()
time.sleep(5)
m.motor_stop()


