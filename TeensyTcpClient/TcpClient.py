#!/usr/bin/env python

import socket
import time


TCP_IP = '192.168.0.102'
TCP_PORT = 80
BUFFER_SIZE = 100
#MESSAGE = "Hello world!"

i = 1
#Do an update of 10-DOF data each second
while 1:
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.connect((TCP_IP, TCP_PORT))
	s.send('1010')
	data_gyro = s.recv(BUFFER_SIZE)
	s.close()
	
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.connect((TCP_IP, TCP_PORT))
	s.send('1011')
	data_accel = s.recv(BUFFER_SIZE)
	s.close()
	
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.connect((TCP_IP, TCP_PORT))
	s.send('1012')
	data_magn = s.recv(BUFFER_SIZE)
	s.close()
	
	print "received response: ",i
	print "Gyroscope: ",			data_gyro
	print "Accelerometer: ",	data_accel
	print "Magnetometer: ",		data_magn
	i = i+1
	time.sleep(1)
