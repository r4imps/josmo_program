#!/usr/bin/env python3
import rospy
import time
import numpy as np

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

    #PAREMA JA VASAKU ENCODERI ANDMETE LUGEMINE    
    def LeftEncoder(self,data):
        self.L_ENCODER = data.data
    def RightEncoder(self,data):
        self.R_ENCODER=data.data
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
        Back2L= 100
        Tl=50
        Tr=50
        #================Encoder Zeroing===================

        #==================ODOMEETRIA ARVUTUSKÄIK====================
        #print(self.L_ENCODER)
        L_Rotation= self.L_ENCODER * ((2*np.pi)/N_tot)
        #print(f"The left wheel rotated: {L_Rotation} degrees")
        R_Rotation= self.R_ENCODER * ((2*np.pi)/N_tot)
        #print(f"The right wheel rotated: {R_Rotation} degrees")
        L_Distance= R*L_Rotation
        #print(f"The left wheel travel: {round(L_Distance,4)} ")
        R_Distance= R*R_Rotation
        #print(f"The righ wheel travel: {round(R_Distance,4)} ")

        Dl=(self.L_ENCODER-(self.L_ENCODER-1)/N_tot)*0.21
        Dr=(self.R_ENCODER-(self.R_ENCODER-1)/N_tot)*0.21
        D= (self.velL-self.velR)/Back2L
        X=((self.velL+self.velR)*np.cos(D))/2
        
        #D=R()
        #Delta_A=(R_Distance+L_Distance)/2
        #print(f"Delta distance:===== {round(Delta_A,3)} =====")

        
        #print(f"Delta rotation: {round(Delta_null,3)}")

        """Delta_x=Delta_A*np.cos(Delta_null)
        #print(f"Delta X:================= {round(Delta_x,2)} ================")
        Delta_y=Delta_A*np.sin(Delta_null)
        #print(f"Delta Y:================= {round(Delta_y,2)} ================")
        Delta_x= round(Delta_x,2)
        Delta_y=round(Delta_y,2)


        return [R_Rotation,L_Rotation,R_Distance,L_Distance,Delta_A,Delta_null,Delta_x,Delta_y]"""
        return[X]


            
