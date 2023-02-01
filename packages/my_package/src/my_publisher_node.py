#!/usr/bin/env python3
import os
import rospy
import numpy as np
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped
from sensor_msgs.msg import Range
import time
from datetime import datetime
from smbus2 import SMBus
speed = WheelsCmdStamped()







class ROSPROG(DTROS):
    def __init__(self, node_name):

        # initialize the DTROS parent class
        super(ROSPROG, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # SUBSCRIBERID JA PUBLISHERID
        self.pub = rospy.Publisher('/josmo/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        rospy.Subscriber('/josmo/front_center_tof_driver_node/range', Range, self.callback)

        rospy.Subscriber('/josmo/right_wheel_encoder_node/tick', WheelEncoderStamped, self.Callback_R_Encoder)
        rospy.Subscriber('/josmo/left_wheel_encoder_node/tick', WheelEncoderStamped, self.Callback_L_Encoder)
        
        
        self.distance=1.0
        self.R_encoder=0
        self.L_encoder=0
        

    def callback(self, data):

        self.distance = data.range

    def Callback_R_Encoder(self,data):
        self.R_encoder = data.data

    def Callback_L_Encoder(self,data):
        self.L_encoder = data.data

    def on_shutdown(self):
        speed.vel_left = 0
        speed.vel_right = 0
        self.pub.publish(speed)
        rospy.on_shutdown()
    
      


    def run(self):
        rate = rospy.Rate(10)
        so=0
        Save_R_en=0
        Display_R_en=0
        Last_R_encoder=0
        Save_L_en=0
        Display_L_en=0
        Last_L_encoder=0

        R_Distance=0
        L_Distance=0
        
        N_tot= 135
        R=0.03424
        baseline_wheel2wheel= 0.1


        while not rospy.is_shutdown():
            read = SMBus(1).read_byte_data(0x3e, 0x11)
            
            L_Rotation= self.L_encoder * ((2*np.pi)/N_tot)
            print(f"The left wheel rotated: {np.rad2deg(L_Rotation)} degrees")
            R_Rotation= self.R_encoder * ((2*np.pi)/N_tot)
            print(f"The right wheel rotated: {np.rad2deg(R_Rotation)} degrees")
            L_Distance= R*L_Rotation
            print(f"The left wheel travel: {round(L_Distance,3)} meters")
            R_Distance= R*R_Rotation
            print(f"The righ wheel travel: {round(R_Distance,3)} meters")
            Distance= (L_Distance+R_Distance)/2
            # encoder value
            print(f"Distance traveled: {round(Distance ,3)} meters")
            """if self.L_encoder>Save_L_en:
                Display_L_en=self.L_encoder-Save_L_en
                

            if self.R_encoder>Save_R_en:
                Display_R_en=self.R_encoder-Save_R_en
            Save_L_en=self.L_encoder
            Save_R_en=self.R_encoder"""
                
        
            #print("SO---"+(str(so)))
            #print("LAST R ENCODER: "+(str(Last_R_encoder)))
            #print("LAST L ENCODER: "+(str(N_tot)))
            #print("SAVE R: "+(str(Save_R_en)))
            #print("SAVE L: "+(str(Save_L_en)))
            print("DISPLAY L ENCODER: "+(str(Display_L_en)))
            print("DISPLAY R ENCODER: "+(str(Display_R_en)))
            #print(self.R_encoder)
            #print(self.L_encoder)
            #print("ToF Distance: "+(str(self.distance)))
            print("Joon"+(str(read)))
            self.pub.publish(speed)

            rate.sleep()

if __name__ == '__main__':
    node = ROSPROG(node_name='my_publisher_node')
    node.run()
    rospy.on_shutdown(ROSPROG.on_shutdown)
    rospy.spin()