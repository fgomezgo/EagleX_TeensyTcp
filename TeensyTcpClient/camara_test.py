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
        rospy.Subscriber('/joy',Joy,self.joyCallback)
        self.x_new,self.y_new,self.stop,self.flag = 0,0,1,0
        self.r = rospy.Rate(5)

    def joyCallback(self,joy):
        self.flag = 0
        self.x_new, self.y_new = joy.axes[0],joy.axes[1]
        if self.x_new < -0.7 :
            self.flag = 1
        if self.x_new > 0.7 :
            self.flag = 2   
        if self.y_new > 0.7 :
            self.flag = 3   
        if self.y_new < -0.7 :
            self.flag = 4

    def driverCam(self):
        
        set_btn = 0
        driver = webdriver.Chrome("/usr/bin/chromedriver")
        driver.get("http://192.168.1.88")
        driver.implicitly_wait(1000)
        self.r.sleep()
        pss = driver.find_element_by_id('passwd')
        pss.send_keys("123qweasd")
        log = driver.find_element_by_id('IDS_LGLOGIN')
        log.click()
        actions = ActionChains(driver)
        
        print "antes del while :D"
        while  not rospy.is_shutdown():
            #print self.x_new,self.y_new
            print self.flag
            if self.flag == 0:
                if set_btn == 1:
                    actions.reset_actions()
                    print "stop"
                    time.sleep(.1)
                    #actions.release(btn).perform()
                    set_btn = 0
            if self.flag == 1:
                if set_btn == 0:
                    print "derecha"
                    time.sleep(.1)
                    btn = driver.find_element_by_id('live_yt5_ptzMoveRight')
                    driver.implicitly_wait(50)
                    set_btn = 1
            if self.flag == 2:
                if set_btn == 0:
                    print "izquieda"
                    time.sleep(.1)
                    btn = driver.find_element_by_id('live_yt5_ptzMoveLeft')
                    driver.implicitly_wait(50)
                    set_btn = 1
            if self.flag == 3:
                if set_btn == 0:
                    print "arriba"
                    time.sleep(.1)
                    btn = driver.find_element_by_id('live_yt1_ptzMoveUp')
                    driver.implicitly_wait(50)
                    set_btn = 1
            if self.flag == 4:
                if set_btn == 0:
                    print "abajo"
                    time.sleep(.1)
                    btn = driver.find_element_by_id('live_yt1_ptzMoveDown')
                    driver.implicitly_wait(50)
                    set_btn = 1
            if set_btn == 1:
                actions.reset_actions()
                time.sleep(0.1)
                actions.move_to_element(btn)
                actions.click(btn)
                actions.pause(0.05)
                actions.release(btn)
                actions.perform()
                time.sleep(.1)
                actions.reset_actions()

            self.r.sleep()

#victor Hurtado 4441271534 llamar por que lo ve igual             
                

if __name__ == '__main__':
    """ main """
    try:
        Joycam = camera()
        Joycam.driverCam()
    except rospy.ROSInterruptException:
		    pass