#!/usr/bin/env python

import rospy
import roslib
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from socket import *
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
        self.rate = rospy.get_param("~rate", 10)
        """
        self.pub_lmotor = rospy.Publisher('left_motor/setpoint', Float64, queue_size=10)
        self.pub_rmotor = rospy.Publisher('right_motor/setpoint', Float64, queue_size=10)
        rospy.Subscriber('cmd_vel', Twist, self.twistCallback)
        """
        
    #############################################################
    def setup(self):
    #############################################################
        r = rospy.Rate(self.rate)
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
        data = bytearray([0x00,0x00,0x00,0x19])
        #print type(data)
        self.socket.sendto(data, self.address) #send command to arduino
        try:
            data, addr = self.socket.recvfrom(10) #Read response from arduino
            #print type(data)
            print data

        except:
            pass
        """        # dx = (l + r) / 2
        # dr = (r - l) / w
            
        self.right = 1.0 * self.dx + self.dr * self.w / 2 
        self.left = 1.0 * self.dx - self.dr * self.w / 2
        # rospy.loginfo("publishing: (%d, %d)", left, right) 
                
        self.pub_lmotor.publish(self.left)
        self.pub_rmotor.publish(self.right)
            
        self.ticks_since_target += 1
        """
    """
    #############################################################
    def twistCallback(self,msg):
    #############################################################
        # rospy.loginfo("-D- twistCallback: %s" % str(msg))
        self.ticks_since_target = 0
        self.dx = msg.linear.x
        self.dr = msg.angular.z
        self.dy = msg.linear.y
    """
    
#############################################################
#############################################################
if __name__ == '__main__':
    """ main """
    try:
        roverComms = RoverComms()
        roverComms.setup()
    except rospy.ROSInterruptException:
			pass