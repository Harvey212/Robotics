#!/usr/bin/env python
import rospy
import time
import sys
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, UInt8, Float64MultiArray
from time import sleep
from tello_driver.msg import TelloStatus

class TelloState:
    
    def __init__(self):
    
        self.height = 0.0
        self.temperature_height_m = 0.0
        self.battery = 100.0
        self.is_flying = False
        self.fly_mode = 999
        self.target_x = -1
        self.target_y = -1
        self.canPass = -1
        #######################################
        self.hasPassedTwice=0
        self.cammiddle=1
        
        self.mission=0  #0: red , 1:blue , 2:findtarget
        #######################################3

class TelloController:

    def move(self, twist, limitTime):
        limitTime = limitTime * 1000
        startTime = int(round(time.time()*1000))
        rate = rospy.Rate(10)
        # print "MOVE~~~~"
        pub_move = rospy.Publisher("/tello/cmd_vel", Twist, queue_size = 10)

        while not rospy.is_shutdown():
          connections = pub_move.get_num_connections()
          if connections > 0:
            endTime = int(round(time.time()*1000))
            if endTime - startTime < limitTime:
              pub_move.publish(twist)

            else:
              #pub_move.publish(Twist())
              break
            rate.sleep() 
    
    def emergency(self):
        rate = rospy.Rate(10)
        
        puber = rospy.Publisher("/tello/emergency", Empty, queue_size=10) 
        
        while not rospy.is_shutdown():
          cons = puber.get_num_connections()
          
          if cons > 0:
            puber.publish(Empty())
            rate.sleep()
            break
        
    def takeoff(self):
        rate = rospy.Rate(10)
        
        puber = rospy.Publisher("/tello/takeoff", Empty, queue_size=10) 
        
        while not rospy.is_shutdown():
          cons = puber.get_num_connections()
          
          if cons > 0:
            puber.publish(Empty())
            rate.sleep()
            break
            
    def land(self):
        rate = rospy.Rate(10)
        
        puber = rospy.Publisher("/tello/land", Empty, queue_size=10) 
        
        while not rospy.is_shutdown():
          cons = puber.get_num_connections()
          
          if cons > 0:
            puber.publish(Empty())
            rate.sleep()
            break
    
    def flip(self, i):
        rate = rospy.Rate(10)
        
        puber = rospy.Publisher("/tello/flip", UInt8, queue_size=10) 
        
        while not rospy.is_shutdown():
          cons = puber.get_num_connections()
          
          if cons > 0:
            puber.publish(UInt8(data=i))
            rate.sleep()
            break
    
class Tello_drone():

    def __init__(self):
        self.state = TelloState()
        self.controler = TelloController()
        self._sensor()
        
    def _sensor(self):
        _ts_sub = rospy.Subscriber("/tello/status", TelloStatus, self._ts_cb, queue_size = 10)
        _tp_sub = rospy.Subscriber("/target_point", Float64MultiArray, self._tp_cb, queue_size = 10)
        
    def _ts_cb(self, data):
        self.state.height = data.height_m
        self.state.temperature_height_m = data.temperature_height_m
        self.state.battery = data.battery_percentage
        self.state.is_flying = data.is_flying
        self.state.fly_mode = data.fly_mode
        
    def _tp_cb(self, data):
        self.state.target_x = data.data[0]
        self.state.target_y = data.data[1]
        self.state.canPass = data.data[2]
        ########################################################
        self.state.hasPassedTwice=data.data[3]
        self.state.cammiddle=data.data[4]
        self.state.mission=data.data[5]
        ###########################################################
