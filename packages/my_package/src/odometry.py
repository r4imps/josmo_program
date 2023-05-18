#!/usr/bin/env python3
import rospy
import time
import numpy as np
from math import sin, cos, pi

from std_msgs.msg import String
from duckietown_msgs.msg import WheelEncoderStamped
from duckietown_msgs.msg import WheelsCmdStamped


global Delta_null
global N_tot
global Delta_A
global Delta_x
global Delta_y
global Back2L
global R
global START




LastDl=0
class ODOMEETRIA():

    def __init__(self):
        rospy.Subscriber('/josmo/left_wheel_encoder_node/tick', WheelEncoderStamped, self.LeftEncoder)
        rospy.Subscriber('/josmo/right_wheel_encoder_node/tick', WheelEncoderStamped, self.RightEncoder)
        rospy.Subscriber('/josmo/wheels_driver_node/wheels_cmd',WheelsCmdStamped,self.Velocity)
        self.L_ENCODER=0
        self.R_ENCODER=0

        self.x = 0
        self.y = 0
        self.theta = 0

        # Set the wheel separation and radius
        self.wheel_sep = 0.1
        self.wheel_radius = 0.03

        # Set the wheel encoder readings
        self.left_ticks = 0
        self.right_ticks = 0
        self.last_left_ticks = 0
        self.last_right_ticks = 0

    #PAREMA JA VASAKU ENCODERI ANDMETE LUGEMINE    
    def LeftEncoder(self,data):
        self.L_ENCODER = data.data
        
    def RightEncoder(self,data):
        self.R_ENCODER=data.data
        self.last_time = rospy.Time.now()
    #VELOCITY
    def Velocity(self,data):
        self.velR=data.vel_right
        self.velL=data.vel_left
    #ALGANDMED ROBOTIST
    
    
    #MUUTUJAD
    flag=1
    Save_L_en=0
    Save_R_en=0
    
    def ODOMETRY_FUNC(self):
        Aeg=time.time()
        

        N_tot= 135
        R=0.03424
        Back2L= 0.1
        Tl=0.05
        Tr=0.05
        #================Encoder Zeroing===================

        #==================ODOMEETRIA ARVUTUSKÄIK====================


        delta_left = self.left_ticks - self.last_left_ticks
        delta_right = self.right_ticks - self.last_right_ticks

        # Calculate the left and right wheel velocities
        vel_left = delta_left / (self.rate.to_sec() * self.wheel_radius)
        vel_right = delta_right / (self.rate.to_sec() * self.wheel_radius)

        # Calculate the robot's linear and angular velocities
        linear_velocity = (vel_left + vel_right) / 2
        angular_velocity = (vel_right - vel_left) / self.wheel_sep

        # Calculate the robot's new position and orientation
        delta_theta = angular_velocity * (self.rate.to_sec())
        delta_x = linear_velocity * cos(self.theta + delta_theta/2) * (self.rate.to_sec())
        delta_y = linear_velocity * sin(self.theta + delta_theta/2) * (self.rate.to_sec())
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Set the last left and right wheel encoder readings
        self.last_left_ticks = self.left_ticks
        self.last_right_ticks = self.right_ticks


        """return [R_Rotation,L_Rotation,R_Distance,L_Distance,Delta_A,Delta_null,Delta_x,Delta_y]"""
        print([self.x, self.y, self.theta])
   

