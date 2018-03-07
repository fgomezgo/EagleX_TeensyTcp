#!/usr/bin/env python

import rospy
import roslib
from socket import *
from datetime import datetime

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

import time 

##############################################################
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
        """ Motor Controllers"""
        rospy.Subscriber('joy', Joy, self.joyCallBack)
        """ IMU """
        self.imu = rospy.Publisher('joint_states', JointState, queue_size=10)
        self.joints = JointState()
        self.joints.name = ['base_link_to_base_pitch', 'base_pitch_to_base_roll', 'base_roll_to_chasis', 'chasis_to_right', 'chasis_to_left', 'chasis_to_right2', 'chasis_to_left2']
        """ Location """
        self.location = rospy.Publisher('rover/coordinates', NavSatFix, queue_size=10)
        #self.pub_rmotor = rospy.Publisher('right_motor/setpoint', Float64, queue_size=10)
        
        
        
    #############################################################
    def setup(self):
    #############################################################
        r = rospy.Rate(self.rate)
        
        """ Drive System Controllers """
        # Left drive system Controllers [0 - 2]
        self.joyL = 0
        self.joyL_old = 0
        self.joyL_change = 0
        # Right drive system Controllers [3 - 5]
        self.joyR = 0
        self.joyR_old = 0
        self.joyR_change = 0
        """ ARM controllers """
        # Shoulder YAW  Controller 6
        self.PLaR = 0
        # Shoulder PITCH Controller 7
        self.L1 = 0 
        self.L2 = 0 
        #  Elbow PITCH Controller 8
        self.R1 = 0
        self.R2 = 0
        """ Gripper Controllers """
        # Wrist PITCH Servo
        self.PUaD = 0
        # Wrist ROLL (Drill) Controller 9
        self.SQ = 0
        self.CI = 0
        # Gripper ROLL (Finger) Controller 10
        self.TR = 0
        self.CR = 0
        """ Cooling System """
        self.SH = 0
        self.OP = 0
        self.cool_left = 0
        self.cool_right = 0
        """ IMU """
        self.time_nsec = 0
        self.jointRB = 0
        self.jointRF = 0
        self.jointLF = 0
        self.jointLB = 0
        self.chassisR = 0
        self.chassisP = 0
        self.chassisY = 0
        """ Location variables """
        self.navsat = NavSatFix()
        self.time_sec = 0
        self.loc_flag = 0

        ###### main loop  ######
        while not rospy.is_shutdown():
            self.synch()
            r.sleep()

    #############################################################
    def synch(self):
    #############################################################
        """
        Escribir algo useful

        """
        ##################### Drive System Controllers #####################
        """ Detect change of joysticks """
        if self.joyL_old != self.joyL:
            self.joyL_change = 1
            self.joyL_old = self.joyL

        if self.joyR_old != self.joyR:
            self.joyR_change = 1
            self.joyR_old = self.joyR

        """ Send current speed """
        if self.joyL_change == 1 or self.joyR_change == 1:
            left_speed = 0
            right_speed = 0
            rospy.loginfo("INFOR: Drive system = Left: %s Right: %s", self.joyL, self.joyR)
            data = bytearray([0x00, 0x00,0x00,0x00])
            """ Set left side speed """
            if self.joyL > 0:
                left_speed = self.joyL
            else:
                left_speed = -self.joyL
                left_speed = left_speed | (1 << 7)
            """ Set right side speed """
            if self.joyR > 0:
                right_speed = self.joyR
            else:
                right_speed = -self.joyR
                right_speed = right_speed | (1 << 7)

            data = bytearray([0x00, left_speed, right_speed, 0x00])

            self.socket.sendto(data, self.address) #send command to arduino

            self.joyL_change = 0
            self.joyR_change = 0

        ##################### ARM Controllers #####################
        """ Shoulder YAW """
        if self.PLaR == 1:
            data = bytearray([0x00,0x00,0x00,0x07])
            self.socket.sendto(data, self.address) #send command to arduino
            rospy.loginfo("INFO: Shoulder moving: LEFT")

        if self.PLaR == -1:
            data = bytearray([0x00,0x00,0x01,0x07])
            self.socket.sendto(data, self.address) #send command to arduino
            rospy.loginfo("INFO: Shoulder moving: RIGHT")

        """ Shoulder PITCH """
        if self.L1 == 1:
            data = bytearray([0x00,0x00,0x00,0x08])
            self.socket.sendto(data, self.address) #send command to arduino
            rospy.loginfo("INFO: Shoulder moving: UP")

        if self.L2 == 1:
            data = bytearray([0x00,0x00,0x01,0x08])
            self.socket.sendto(data, self.address) #send command to arduino
            rospy.loginfo("INFO: Shoulder moving: DOWN")
        
        """ Elbow PITCH """
        if self.R1 == 1:
            data = bytearray([0x00,0x00,0x00,0x09])
            self.socket.sendto(data, self.address) #send command to arduino
            rospy.loginfo("INFO: Elbow moving: UP")

        if self.R2 == 1:
            data = bytearray([0x00,0x00,0x01,0x09])
            self.socket.sendto(data, self.address) #send command to arduino
            rospy.loginfo("INFO: Elbow moving: DOWN")

        ##################### Gripper Controllers #####################
        """ Wrist PITCH """
        if self.PUaD == 1:
            data = bytearray([0x00,0x00,0x00,0x0A])
            self.socket.sendto(data, self.address) #send command to arduino
            rospy.loginfo("INFO: Wrist moving: UP")

        if self.PUaD == -1:
            data = bytearray([0x00,0x00,0x01,0x0A])
            self.socket.sendto(data, self.address) #send command to arduino
            rospy.loginfo("INFO: Wrist moving: DOWN")

        """ Wrist ROLL (Drill) """
        if self.SQ == 1:
            data = bytearray([0x00,0x00,0x00,0x0B])
            self.socket.sendto(data, self.address) #send command to arduino
            rospy.loginfo("INFO: Wrist moving: ROLL LEFT")

        if self.CI == 1:
            data = bytearray([0x00,0x00,0x01,0x0B])
            self.socket.sendto(data, self.address) #send command to arduino
            rospy.loginfo("INFO: Wrist moving: ROLL RIGHT")

        """ Gripper ROLL """
        if self.TR == 1:
            data = bytearray([0x00,0x00,0x00,0x0C])
            self.socket.sendto(data, self.address) #send command to arduino
            rospy.loginfo("INFO: Gripper moving: OPENING")

        if self.CR == 1:
            data = bytearray([0x00,0x00,0x01,0x0C])
            self.socket.sendto(data, self.address) #send command to arduino
            rospy.loginfo("INFO: Gripper moving: CLOSING")

        ##################### Cooling System #####################
        if self.OP == 1:
            self.cool_left ^= 1
            data = bytearray([0x00, 0x00, (self.cool_left << 1) | self.cool_right, 0x0D])
            self.socket.sendto(data, self.address) #send command to arduino
        if self.SH == 1:
            self.cool_right ^= 1
            data = bytearray([0x00, 0x00, (self.cool_left << 1) | self.cool_right, 0x0D])
            self.socket.sendto(data, self.address) #send command to arduino

        ##################### IMU #####################
        
        #if (rospy.Time.now().secs - self.time_nsec) >= 1:
            
        data = bytearray([0x00, 0x00, 0x00, 0x0E])  # Suspension Right Back
        self.socket.sendto(data, self.address) #send command to arduino
        try:
            data, addr = self.socket.recvfrom(15) #Read response from arduino
            self.jointRB = data
        except:
            pass

        data = bytearray([0x00, 0x00, 0x00, 0x4E])  # Suspension Right Front
        self.socket.sendto(data, self.address) #send command to arduino
        try:
            data, addr = self.socket.recvfrom(15) #Read response from arduino
            self.jointRF = data
        except:
            pass

        data = bytearray([0x00, 0x00, 0x00, 0x8E])  # Suspension Left Front
        self.socket.sendto(data, self.address) #send command to arduino
        try:
            data, addr = self.socket.recvfrom(15) #Read response from arduino
            self.jointLF = data
        except:
            pass

        data = bytearray([0x00, 0x00, 0x00, 0xCE])  # Suspension Left Back
        self.socket.sendto(data, self.address) #send command to arduino
        try:
            data, addr = self.socket.recvfrom(15) #Read response from arduino
            self.jointLB = data
        except:
            pass

        data = bytearray([0x00, 0x00, 0x00, 0x0F])  # Suspension Left Back
        self.socket.sendto(data, self.address) #send command to arduino
        try:
            data, addr = self.socket.recvfrom(15) #Read response from arduino
            self.chassisR = data
        except:
            pass

        data = bytearray([0x00, 0x00, 0x00, 0x4F])  # Suspension Left Back
        self.socket.sendto(data, self.address) #send command to arduino
        try:
            data, addr = self.socket.recvfrom(15) #Read response from arduino
            self.chassisP = data
        except:
            pass

        data = bytearray([0x00, 0x00, 0x00, 0x8F])  # Suspension Left Back
        self.socket.sendto(data, self.address) #send command to arduino
        try:
            data, addr = self.socket.recvfrom(15) #Read response from arduino
            self.chassisY = data
        except:
            pass

        #print "Acel: " + self.jointRB + " " + self.jointRF +" "+ self.jointLF + " " + self.jointLB +" "+ self.chassisP + " " + self.chassisR + " " + self.chassisY 

        self.joints.header = Header()
        self.joints.header.stamp = rospy.Time.now()

        #self.joints.position = [float(self.chassisP), float(self.chassisR), float(self.chassisY), float(self.jointRB), float(self.jointLB), float(self.jointRF), float(self.jointLF)]
        self.joints.position = [0, 0, 0, float(self.jointRB), float(self.jointLB), float(self.jointRF), float(self.jointLF)]
        self.joints.velocity = []
        self.joints.effort = []
        self.imu.publish(self.joints)


        self.time_nsec = rospy.Time.now().secs

        ##################### Location #####################
        if (rospy.Time.now().secs - self.time_sec) >= 2:
            rospy.loginfo("INFO: Location: Query")

            """ Get Latitude """
            data = bytearray([0x00,0x00,0x00,0x11])
            self.socket.sendto(data, self.address) #send command to arduino
            try:
                data, addr = self.socket.recvfrom(15) #Read response from arduino
                if data == "-1":
                    self.loc_flag = 1
                else:
                    self.navsat.latitude = float(data)
                    #print data
            except:
                pass

            """ Get Longitude """
            data = bytearray([0x00,0x00,0x00,0x51])
            self.socket.sendto(data, self.address) #send command to arduino
            try:
                data, addr = self.socket.recvfrom(15) #Read response from arduino
                if data == "-1" or self.loc_flag == 1:
                    rospy.loginfo("WARNING: No FIX, Dropping frame")
                    self.loc_flag = 0
                else:
                    self.navsat.longitude = float(data)
                    rospy.loginfo("INFO: Location: Recieved")
                    #print data
            except:
                pass

            self.location.publish(self.navsat)
            self.time_sec = rospy.Time.now().secs
        
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
        
        # Left drive system Controllers [0 - 2]
        self.joyL = int(round(data.axes[1] * 100))
        # Right drive system Controllers [3 - 5]
        self.joyR = int(round(data.axes[4] * 100))
        # Shoulder YAW  Controller 6
        self.PLaR = data.axes[6]
        # Shoulder PITCH Controller 7
        self.L1 = data.buttons[4] 
        self.L2 = data.buttons[6]
        #  Elbow PITCH Controller 8
        self.R1 = data.buttons[5]
        self.R2 = data.buttons[7]
        # Wrist PITCH Servo
        self.PUaD = data.axes[7]
        # Wrist ROLL (Drill) Controller 9
        self.SQ = data.buttons[3]
        self.CI = data.buttons[1]
        # Gripper ROLL (Finger) Controller 10
        self.TR = data.buttons[2]
        self.CR = data.buttons[0]
        # Cooling System (Relays)
        self.SH = data.buttons[8]
        self.OP = data.buttons[9]

        
    
#############################################################
#############################################################
if __name__ == '__main__':
    """ main """
    try:
        roverComms = RoverComms()
        roverComms.setup()
    except rospy.ROSInterruptException:
			pass