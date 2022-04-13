#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from serial import Serial
import serial.tools.list_ports
from sensor_msgs.msg import BatteryState
import time
import threading

x=serial.Serial('/dev/battery',9600)
def faSong():   # 发送函数
    while True: # 循环发送数据
          
        myinput=b'\xDD\xA5\x03\x00\xFF\xFD\x77'    # 需要发送的十六进制数据
        x.write(myinput)    # 用write函数向串口发送数据
        time.sleep(0.5)      # 设置发送间隔时间
    #print(myinput)
def jieShou():  # 接收函数
    while True: # 循环接收数据
        while x.inWaiting()>0:  # 当接收缓冲区中的数据不为零时，执行下面的代码
            myout=x.read(34) # 提取接收缓冲区中的前7个字节数
            #print (myout)
            checksum=~(myout[2]+myout[3]+myout[4]+myout[5]+myout[6]+myout[7]+myout[8]+myout[9]+myout[10]+myout[11]
                       +myout[12]+myout[13]+myout[14]+myout[15]+myout[16]+myout[17]+myout[18]+myout[19]+myout[20]
                       +myout[21]+myout[22]+myout[23]+myout[24]+myout[25]+myout[26]+myout[27]+myout[28]+myout[29]
                       +myout[30])+1
            checksum_high=(checksum>>8)&0xff
            checksum_low=checksum&0xff
            if checksum_high==myout[31] and checksum_low==myout[32]:
               data=myout
               #print(data)  
            #return data
            datas=''.join(map(lambda x:('/x' if len(hex(x))>=4 else '/x0')+hex(x)[2:],data))
            #print(datas)
            new_datas=datas[2:].split('/x') # 由于datas变量中的数据前两个是/x，所以用到切片工具
            #need=''.join(new_datas) # 将前面的列表中的数据连接起来
           
            #print(new_datas)
            current=new_datas[6]+new_datas[7]
            current=float(hex2dec(current))/100    #10mA
            if current>0:
               state=1
            else:
               state=3
            #print(current)
            rsoc=float(hex2dec(new_datas[23]))/100 #%
            #state=int(hex2dec(new_datas[24]))
            #print(rsoc)
            value=BatteryState()
            value.current=current
            value.percentage=rsoc
            value.power_supply_status=state
            global cmd_pub
            cmd_pub.publish(value)

def hex2dec(string_num) :
    width=16
    dec_data=int(string_num,16)
    if dec_data >2**(width-1)-1:
       dec_data=2**width-dec_data
       dec_data=0-dec_data
    return dec_data
          
if __name__ == '__main__':
    rospy.init_node('batterypublisher')
    rospy.sleep(5)
    global cmd_pub
    cmd_pub = rospy.Publisher('battery',BatteryState,queue_size=1)
    t1=threading.Thread(target=jieShou)
    t2=threading.Thread(target=faSong)
    t2.start()
    t1.start()

