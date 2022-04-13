#!/usr/bin/env python
# -*- coding: utf-8 -*-
from socket import *
from time import ctime
import time
import rospy

HOST = '192.168.1.102'
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
tcpSerSock.listen(5) #监听连接，同时连接请求的最大数目


print('等待客户机的连接')
tcpCliSock, addr = tcpSerSock.accept()  #接收继电器端连接请求
print('连接成功')
print('客户端IP与端口如下:')
print(addr)
while not rospy.is_shutdown():
  print('TCP-KP系列网络继电器功能测试：')

  print('控制继电器通道1常开吸合发送：AT+STACH1=1\\r\\n')
  meg = "AT+STACH1=1\n"
  tcpCliSock.send(meg.encode())
  recv_data = tcpCliSock.recv(BUFSIZ)
  print('网络继电器应答：')
  print(recv_data.decode('gbk'))
  
  print('指令执行成功!')
  time.sleep(1)#休眠0.5秒
#================================================
  print('获取开关量输入接口1的状态发送：AT+OCCH1=?\\r\\n')
  meg = "AT+OCCH1=?\r\n"
  tcpCliSock.send(meg.encode())
  recv_data0 = tcpCliSock.recv(BUFSIZ)
  print('网络继电器应答：')
  #print(recv_data.replace("\r\n", "\\r\\n").decode('gbk'))
  print(recv_data0)
  recv_data1 = (recv_data0.split(':')[1])
  print(recv_data1)
  #recv_data2 = int(recv_data1[1])
  print('剪切后输出！')
  #recv_data3 = recv_data2[0]
  #print(recv_data2)
  print('指令执行成功!')
  time.sleep(1)#休眠0.5秒

#================================================
  print('控制继电器通道1常开断开发送：AT+STACH1=0\\r\\n')
  meg = "AT+STACH1=0\r\n"
  tcpCliSock.send(meg.encode())
  recv_data = tcpCliSock.recv(BUFSIZ)
  print('网络继电器应答：')
  print(recv_data.decode('gbk'))
  print('指令执行成功!')
  time.sleep(1)#休眠0.5秒

#================================================
#print('控制继电器通道1常开接口吸合15秒后断开发送：AT+STACH1=1,15\\r\\n')
#meg = "AT+STACH1=1,15\r\n"
#tcpCliSock.send(meg.encode())
#recv_data = tcpCliSock.recv(BUFSIZ)
#print('网络继电器应答：')
#print(recv_data.decode('gbk'))
#print('指令执行成功!')
#time.sleep(0.5)#休眠0.5秒


  
#print('关闭客户端连接！')
#tcpCliSock.close() #关闭与继电器的连接
#tcpSerSock.close() #关闭服务器socket
#print('测试完成！')


