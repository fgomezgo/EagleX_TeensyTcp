#!/usr/bin/env python

import rospy
import roslib
from socket import *
from datetime import datetime

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from sensor_msgs.msg import NavSatFix

import time 

#############################################################
#############################################################
class RoverComms():
#############################################################
#############################################################

    #############################################################
    def __init__(self):
    #############################################################
        rospy.init_node("rover_comms")
        nodename = rospy.get_name()
        rospy.loginfo("%s started" % nodename)
        # Set server ip address, timeout and ROS rate here
        self.address = rospy.get_param("~address",('192.168.1.200', 5000))
        rospy.loginfo("Starting socket at: %s" % self.address[0])
        self.socket = socket(AF_INET, SOCK_DGRAM)
        self.socket.settimeout(1)
        self.rate = rospy.get_param("~rate", 5)
        
        self.location = rospy.Publisher('rover/coordinates', NavSatFix, queue_size=10)
        #self.pub_rmotor = rospy.Publisher('right_motor/setpoint', Float64, queue_size=10)
        rospy.Subscriber('joy', Joy, self.joyCallBack)
        
        
    #############################################################
    def setup(self):
    #############################################################
        r = rospy.Rate(self.rate)
        self.navsat = NavSatFix()
        self.but2 = 0 
        self.but3 = 0 
        self.but4 = 0 
        self.but5 = 0 
        ###### main loop  ######
        while not rospy.is_shutdown():
            self.synch()
            r.sleep()

    #############################################################
    def synch(self):
    #############################################################
        """
        Byte array with device and isntruction Ids
        [0x00,0x00,0x00,0x19] is the isntruction for obtaining the GPS lat/long
        [0x98,0x76,0x54,0x32] is just an example 
        """
        if self.but2 == 1:
            data = bytearray([0x00,0x00,0x00,0x07])
            self.socket.sendto(data, self.address) #send command to arduino
            rospy.loginfo("self.but2")

        if self.but4 ==1:
            data = bytearray([0x00,0x00,0x02,0x07])
            self.socket.sendto(data, self.address) #send command to arduino
            rospy.loginfo("self.but4")
        """
        #print type(data)
        self.socket.sendto(data, self.address) #send command to arduino
        try:
            data, addr = self.socket.recvfrom(15) #Read response from arduino
            #print type(data)
            if data == "-1":
                rospy.loginfo("WARNING: No FIX, Dropping frame")
            else:
                self.navsat.latitude = float(data)
                #print data
        except:
            pass

        
        data = bytearray([0x00,0x00,0x00,0x1A])
        #print type(data)
        self.socket.sendto(data, self.address) #send command to arduino
        try:
            data, addr = self.socket.recvfrom(15) #Read response from arduino
            #print type(data)
            if data == "-1":
                rospy.loginfo("WARNING: No FIX, Dropping frame")
            else:
                self.navsat.longitude = float(data)
                #print data
        except:
            pass

        self.location.publish(self.navsat)
        """
        """        # dx = (l + r) / 2
        # dr = (r - l) / w
            
        self.right = 1.0 * self.dx + self.dr * self.w / 2 
        self.left = 1.0 * self.dx - self.dr * self.w / 2
        # rospy.loginfo("publishing: (%d, %d)", left, right) 
                
        self.pub_lmotor.publish(self.left)
        self.pub_rmotor.publish(self.right)
            
        self.ticks_since_target += 1
        """
    #############################################################
    def joyCallBack(self, data):
    #############################################################
        # rospy.loginfo("-D- twistCallback: %s" % str(msg))
        self.but2 = data.buttons[1]
        self.but3 = data.buttons[2]
        self.but4 = data.buttons[3]
        self.but5 = data.buttons[4]

        
    
#############################################################
#############################################################
if __name__ == '__main__':
    """ main """
    try:
        roverComms = RoverComms()
        roverComms.setup()
    except rospy.ROSInterruptException:
			pass