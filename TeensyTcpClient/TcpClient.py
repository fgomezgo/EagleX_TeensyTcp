#!/usr/bin/env python

import socket


TCP_IP = '192.168.0.102'
TCP_PORT = 80
BUFFER_SIZE = 1024
MESSAGE = "Hello world!"

i = 1

while 1:
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.connect((TCP_IP, TCP_PORT))
	s.send(MESSAGE)
	data = s.recv(BUFFER_SIZE)
	s.close()

	print "received data: ",i," ",data
	i = i+1
