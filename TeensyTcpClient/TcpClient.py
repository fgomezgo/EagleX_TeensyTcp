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
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

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
        """ PS4 Controlllers"""
        rospy.Subscriber('joy_PS4/joy', Joy, self.joyPS4Callback)
        """ Mad Cat Controlllers"""
        rospy.Subscriber('joy_MCJ/joy', Joy, self.joyMCJCallBack)

        """ IMU """
        self.imu = rospy.Publisher('joint_states', JointState, queue_size=10)
        self.joints = JointState()
        self.joints.name = ['base_link_to_base_pitch', 'base_pitch_to_base_roll', 'base_roll_to_chasis', 'chasis_to_right', 'chasis_to_left', 'chasis_to_right2', 'chasis_to_left2']
        """ Location """
        self.location = rospy.Publisher('rover/coordinates', NavSatFix, queue_size=10)
        #self.pub_rmotor = rospy.Publisher('right_motor/setpoint', Float64, queue_size=10)
        """ Status """
        self.current_status = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)

        
        
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
        # Shoulder YAW  Controller 4
        self.PLaR = 0
        self.PLaR_old = 0
        self.PLaR_change = 0
        self.PLaR_speed = 0

        # Shoulder PITCH Controller 6
        self.L1 = 0 
        self.L2 = 0 
        self.L1_old = 0 
        self.L2_old = 0 
        self.L1_change = 0 
        self.L2_change = 0
        self.L1_L2_speed = 0

        #  Elbow PITCH Controller 5
        self.R1 = 0
        self.R2 = 0
        self.R1_old = 0
        self.R2_old = 0
        self.R1_change = 0
        self.R2_change = 0
        self.R1_R2_speed = 0

        """ Gripper Controllers """
        # Wrist PITCH Servo
        self.PUaD = 0
        self.PUaD_old = 0
        self.PUaD_change = 0
        self.PUaD_speed = 0
        self.PUaD_trigger = 0

        # Wrist ROLL (Drill) Controller 7
        self.SQ = 0
        self.CI = 0
        self.SQ_old = 0
        self.CI_old = 0
        self.SQ_change = 0
        self.CI_change = 0
        self.WRIST_ROLL_speed = 0

        # Gripper ROLL (Finger) Controller 9
        self.TR = 0
        self.CR = 0
        self.TR_old = 0
        self.CR_old = 0
        self.TR_change = 0
        self.CR_change = 0
        self.GRIPPER_ROLL_speed = 0

        
        # Joystick
        self.mdj_2 = 0
        self.mdj_6 = 0
        self.mdj_7 = 0
        self.mdj_8 = 0
        self.mdj_9 = 0

        self.mdj_2_change = 0
        self.mdj_6_change = 0
        self.mdj_7_change = 0
        self.mdj_8_change = 0
        self.mdj_9_change = 0

        self.mdj_2_old = 0
        self.mdj_6_old = 0
        self.mdj_7_old = 0
        self.mdj_8_old = 0
        self.mdj_9_old = 0

        self.science_Revolver = 0


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
        self.teporocho_2 = ""
        self.teporocho_2_h = ""
        self.teporocho_2_t= ""
        self.teporocho_2_f = ""
        self.teporocho_2_k= ""
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
        ##################### Status #####################
        """ Create blank diagnostics """
        self.generic_diagnostic_array = DiagnosticArray()
        self.generic_diagnostic_status = DiagnosticStatus()
        self.generic_key_value = KeyValue()
        # Generate header
        self.generic_diagnostic_array.header = Header()

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
            try:
                data, addr = self.socket.recvfrom(25) #Read response from arduino
                self.generic_diagnostic_status.level = 0
                self.generic_diagnostic_status.name = 'Motor Controllers  avg V/Temp'
                self.generic_diagnostic_status.message = 'Average voltage and temperature of drivers'
                self.generic_diagnostic_status.hardware_id = '1'
                # Add values
                self.sh_yaw_temp_key_value = KeyValue()
                self.sh_yaw_temp_key_value.key = 'AVG Voltage:'
                self.sh_yaw_temp_key_value.value = str(float(data[0:3])/10.0) + ' C'
                self.generic_diagnostic_status.values.append(self.sh_yaw_temp_key_value)

                self.sh_pitch_temp_key_value = KeyValue()
                self.sh_pitch_temp_key_value.key = 'AVG Temp:'
                self.sh_pitch_temp_key_value.value = str(float(data[3:8])/1000.0) + ' V'
                self.generic_diagnostic_status.values.append(self.sh_pitch_temp_key_value)

                #Append other diagnostics
                self.generic_diagnostic_array.status.append(self.generic_diagnostic_status)
            except:
                pass
            self.joyL_change = 0
            self.joyR_change = 0

        ##################### ARM Controllers #####################
        """ Shoulder YAW """
        if self.PLaR_old != self.PLaR:
            self.PLaR_change = 1
            self.PLaR_old = self.PLaR
                 
        if self.PLaR_change == 1:
            if self.PLaR == -1:
                self.PLaR_speed = 100
                rospy.loginfo("INFO: Shoulder moving: LEFT ")
            
            if self.PLaR == 1:
                self.PLaR_speed = 100 | (1 << 7)
                rospy.loginfo("INFO: Shoulder moving: RIGHT ")
            
            if self.PLaR == 0:
                self.PLaR_speed = 0
            
        """ Shoulder PITCH """
        if  self.L1_old != self.L1:
            self.L1_change = 1
            self.L1_old = self.L1
        
        if  self.L2_old != self.L2:
            self.L2_change = 1
            self.L2_old = self.L2

        if self.L1_change == 1:
            if self.L1 == 1:
                self.L1_L2_speed = 80 
                rospy.loginfo("INFO: Shoulder moving: DOWN ")
            else:
                self.L1_L2_speed = 0
        
        if self.L2_change == 1 :
            if self.L2 == 1 :
                self.L1_L2_speed = 80 | (1 << 7)
                rospy.loginfo("INFO: Shoulder moving: UP")
            else:
                self.L1_L2_speed = 0

        """ Elbow PITCH """
        if  self.R1_old != self.R1:
            self.R1_change = 1
            self.R1_old = self.R1
        
        if  self.R2_old != self.R2:
            self.R2_change = 1
            self.R2_old = self.R2

        if self.R1_change == 1:
            if self.R1 == 1:
                self.R1_R2_speed = 80 | (1 << 7)
                rospy.loginfo("INFO: Shoulder moving: DOWN ")
            else:
                self.R1_R2_speed = 0
        
        if self.R2_change == 1 :
            if self.R2 == 1 :
                self.R1_R2_speed = 80 
                rospy.loginfo("INFO: Shoulder moving: UP")
            else:
                self.R1_R2_speed = 0 

        if (self.PLaR_change == 1) or (self.L1_change == 1) or (self.L2_change == 1) or (self.R1_change == 1) or (self.R2_change == 1):
            # Print speed
            rospy.loginfo("INFO: Shoulder yaw: %s Shoulder pitch: %s Elbow pitch: %s", self.PLaR_speed, self.L1_L2_speed, self.R1_R2_speed)
            # Concatenate values
            data = bytearray([self.R1_R2_speed, self.L1_L2_speed, self.PLaR_speed, 0x07])
            self.socket.sendto(data, self.address) #send command to arduino
            # Reset changes
            self.PLaR_change = 0
            self.L1_change = 0
            self.L2_change = 0
            self.R1_change = 0
            self.R2_change = 0

        ##################### Gripper Controllers #####################
        """ Wrist PITCH """
        if self.PUaD == -1:
            self.PUaD_speed = 0x01
            self.PUaD_change = 1
            self.PUaD_trigger = 0
            rospy.loginfo("INFO: Wrist moving: DOWN ")
        
        if self.PUaD == 1:
            self.PUaD_speed = 0x02
            self.PUaD_change = 1
            self.PUaD_trigger = 0
            rospy.loginfo("INFO: Wrist moving: UP ")
        
        if self.PUaD == 0:
            self.PUaD_speed = 0x00
            if(self.PUaD_trigger == 0):
                self.PUaD_change = 1
                self.PUaD_trigger = 1
            else:
                self.PUaD_change = 0

        """ Science Revolver Servo """

        # Button 6
        if  self.mdj_6_old != self.mdj_6:
            self.mdj_6_change = 1
            self.mdj_6_old = self.mdj_6

        # Button 7
        if  self.mdj_7_old != self.mdj_7:
            self.mdj_7_change = 1
            self.mdj_7_old = self.mdj_7

        # Button 8
        if  self.mdj_8_old != self.mdj_8:
            self.mdj_8_change = 1
            self.mdj_8_old = self.mdj_8
        
        # Button 9
        if  self.mdj_9_old != self.mdj_9:
            self.mdj_9_change = 1
            self.mdj_9_old = self.mdj_9

        if self.mdj_6_old == 1:
            if self.mdj_6 == 1:
                self.science_Revolver = 0x04
                rospy.loginfo("INFO: Sample collector POS 0")

        if self.mdj_7_old == 1:
            if self.mdj_7 == 1:
                self.science_Revolver = 0x08
                rospy.loginfo("INFO: Sample collector POS 1")

        if self.mdj_8_old == 1:
            if self.mdj_8 == 1:
                self.science_Revolver = 0x10
                rospy.loginfo("INFO: Sample collector POS 2")

        if self.mdj_9_old == 1:
            if self.mdj_9 == 1:
                self.science_Revolver = 0x20
                rospy.loginfo("INFO: Sample collector POS 3")

        """ Wrist ROLL (Drill) """
        if  self.SQ_old != self.SQ:
            self.SQ_change = 1
            self.SQ_old = self.SQ
        
        if  self.CI_old != self.CI:
            self.CI_change = 1
            self.CI_old = self.CI

        if self.SQ_change == 1:
            if self.SQ == 1:
                self.WRIST_ROLL_speed = 40 | (1 << 7)
                rospy.loginfo("INFO: Wrist moving: ROLL LEFT")
            else:
                self.WRIST_ROLL_speed = 0
        
        if self.CI_change == 1 :
            if self.CI == 1 :
                self.WRIST_ROLL_speed = 40 
                rospy.loginfo("INFO: Wrist moving: ROLL RIGHT")
            else:
                self.WRIST_ROLL_speed = 0 
            

        """ Gripper ROLL """
        if  self.TR_old != self.TR:
            self.TR_change = 1
            self.TR_old = self.TR
        
        if  self.CR_old != self.CR:
            self.CR_change = 1
            self.CR_old = self.CR

        if self.TR_change == 1:
            if self.TR == 1:
                self.GRIPPER_ROLL_speed = 100 | (1 << 7)
                rospy.loginfo("INFO: Gripper moving: OPENING")
            else:
                self.GRIPPER_ROLL_speed = 0
        
        if self.CR_change == 1 :
            if self.CR == 1 :
                self.GRIPPER_ROLL_speed = 100 
                rospy.loginfo("INFO: Gripper moving: CLOSING")
            else:
                self.GRIPPER_ROLL_speed = 0 
            
        if (self.PUaD_change == 1) or (self.SQ_change == 1) or (self.CI_change == 1) or (self.TR_change == 1) or (self.CR_change == 1) or (self.mdj_6_change == 1) or (self.mdj_7_change == 1) or (self.mdj_8_change == 1) or  (self.mdj_9_change == 1):
            # Print speed
            #rospy.loginfo("INFO: Wrist yaw: %s Wrist roll: %s Gripper Roll: %s", self.PUaD_speed, self.WRIST_ROLL_speed, self.GRIPPER_ROLL_speed)
            
            # Concatenate values
            self.PUaD_speed = self.PUaD_speed | self.science_Revolver
            rospy.loginfo("Holo: %s", self.PUaD_speed)
            data = bytearray([self.GRIPPER_ROLL_speed, self.WRIST_ROLL_speed, self.PUaD_speed, 0x08])
            self.socket.sendto(data, self.address) #send command to arduino
            # Reset changes
            self.PUaD_change = 0
            self.SQ_change = 0
            self.CI_change = 0
            self.TR_change = 0
            self.CR_change = 0
            
            self.mdj_6_change = 0
            self.mdj_7_change = 0
            self.mdj_8_change = 0
            self.mdj_9_change = 0
#TEPOROCHO#######################################################################################
        # Button 2###
        if(self.mdj_2_old != self.mdj_2):
            self.mdj_2_change = 1
            self.mdj_2_old = self.mdj_2

        # Data teporocho #################################################################################################
        if (self.mdj_2_change == 1) :
            if (self.mdj_2 == 1 ):
                # Concatenate values
                data = bytearray([0x00,0x00,0x00, 0x06])
                self.socket.sendto(data, self.address) #send command to arduino
            # Reset changes
            self.mdj_2_change = 0
            """"""""""""""""""""""""""""""""""""
            try:
                data, addr = self.socket.recvfrom(15) #Read response from arduino
                self.teporocho_2 = data
                print data
            except:
                pass
            #self.teporocho_2_h , self.teporocho_2_t , self.teporocho_2_f , self.teporocho_2_k = self.teporocho_2.split(",")
            #pint (" Humedad " + self.teporocho_2_h + "% ," + self.teporocho_2_t + " C ," + self.teporocho_2_f + " F ," + self.teporocho_2_k + " K " )
        ##################### Cooling System #####################
        if self.OP == 1:
            self.cool_left ^= 1
            data = bytearray([0x00, 0x00, (self.cool_left << 1) | self.cool_right, 0x0D])
            self.socket.sendto(data, self.address) #send command to arduino
        if self.SH == 1:
            self.cool_right ^= 1
            data = bytearray([0x00, 0x00, (self.cool_left << 1) | self.cool_right, 0x0D])
            self.socket.sendto(data, self.address) #send command to arduino

  
     
        ##################### Location #####################
        """
        if (rospy.Time.now().secs - self.time_sec) >= 2:
            rospy.loginfo("INFO: Location: Query")

            # Get Latitude 
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

            # Get Longitude 
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
        
        # Publish 
        self.current_status.publish(self.generic_diagnostic_array)
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
    def joyPS4Callback(self, data):
    #############################################################
        
        # Left drive system Controllers [0 - 2]
        self.joyL = int(round(data.axes[1] * 100))
        # Right drive system Controllers [3 - 5]
        self.joyR = int(round(data.axes[4] * 100))
        # Shoulder YAW  Controller 6
        self.PLaR = int(data.axes[6])
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
    def joyMCJCallBack(self, data):
    #############################################################
        self.mdj_2 = data.buttons[1]
        self.mdj_6 = data.buttons[5]
        self.mdj_7 = data.buttons[6]
        self.mdj_8 = data.buttons[7]
        self.mdj_9 = data.buttons[8]
#############################################################
#############################################################
if __name__ == '__main__':
    """ main """
    try:
        roverComms = RoverComms()
        roverComms.setup()
    except rospy.ROSInterruptException:
			pass