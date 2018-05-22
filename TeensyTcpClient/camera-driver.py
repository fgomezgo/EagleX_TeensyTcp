#!/usr/bin/env python
from selenium import webdriver
from selenium.webdriver.common.keys import Keys
from selenium.common.exceptions import NoSuchElementException
from selenium.webdriver import ActionChains
from selenium.webdriver.common.by import By
from selenium.webdriver.common.keys import Keys
import time
import rospy
from sensor_msgs.msg import Joy

class camera():
    def __init__(self):
        rospy.init_node("camera")
        nodename = rospy.get_name()
        rospy.Subscriber('/joy',Joy,self.joyCallback)
        self.x_new,self.y_new = 0,0
        self.stop=0

    def joyCallback(self,joy):
        self.x_new, self.y_new = joy.axes[0],joy.axes[1]
        print self.x_new, self.y_new
        if self.x_new == 0 and self.y_new == 0:
            print "Hola callback"
            self.stop=1

        

    def driverCam(self):
        
        driver = webdriver.Chrome("/usr/bin/chromedriver")
        driver.get("http://192.168.1.88")
        r = rospy.Rate(5)
        r.sleep()
        pss = driver.find_element_by_id('passwd')
        pss.send_keys("123qweasd")
        log = driver.find_element_by_id('IDS_LGLOGIN')
        log.click()

        actions=ActionChains(driver)

        x_old,x_change,y_old,y_change,flag = 0,0,0,0,0
        print "antes del while :D"


        while  not rospy.is_shutdown():
            if self.x_new != x_old :
                x_change = 1
                x_old = self.x_new
            if self.y_new != y_old :
                y_change = 1
                y_old = self.y_new
            
            if x_change == 1 or y_change == 1:

                if self.x_new < -0.7 :
                    button = driver.find_element_by_id('live_yt5_ptzMoveRight')
                    flag = 1
                

                if self.x_new > 0.7 :
                    button = driver.find_element_by_id('live_yt5_ptzMoveLeft')
                    flag = 1
                
            
                if self.y_new > 0.7 :
                    button = driver.find_element_by_id('live_yt1_ptzMoveUp')
                    flag = 1
                
            
                if self.y_new < -0.7 :
                    button = driver.find_element_by_id('live_yt1_ptzMoveDown')
                    flag = 1
                
                if flag == 1:
                    button.click()
                    if self.stop == 1:
                        print "Hola"
                        actions.release(button).perform()
                        self.stop=0
                        flag = 0

                x_change,y_change = 0,0
                    
                    
      
        
            r.sleep()
            
                

if __name__ == '__main__':
    """ main """
    try:
        Joycam = camera()
        Joycam.driverCam()
    except rospy.ROSInterruptException:
		    pass