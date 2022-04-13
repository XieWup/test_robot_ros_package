#!/usr/bin/env python
# -*- coding: utf-8 -*-
from socket import *
from time import ctime
import time

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
tcpSerSock.listen(5) #监听连接，同时连接请求的最大数目


print('等待客户机的连接')
tcpCliSock, addr = tcpSerSock.accept()  #接收继电器端连接请求
print('连接成功')
print('客户端IP与端口如下:')
print(addr)

