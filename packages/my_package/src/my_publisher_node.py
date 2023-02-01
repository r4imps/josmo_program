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
        self.sec=0
        

    def callback(self, data):

        self.distance = data.range


    def Callback_R_Encoder(self,data):
        self.R_encoder = data.data
        self.sec= data.header.seq

    def Callback_L_Encoder(self,data):
        self.L_encoder = data.data

    def on_shutdown(self):
        speed.vel_left = 0
        speed.vel_right = 0
        Save_L_en=0
        Save_R_en=0
        self.pub.publish(speed)
        rospy.on_shutdown()
    
      


    def run(self):
        rate = rospy.Rate(10)
        flag=1
        so=0

        Save_L_en=0
        Save_R_en=0
        Display_L_en=0
        Display_R_en=0

        Sec_save=0
  

        Dst_Save=0
        
        R_Distance=0
        L_Distance=0
        Save_R_deg=0
        Save_L_deg=0

####################
        N_tot= 135
        R=0.03424
        baseline_wheel2wheel= 0.1
        
        Delta_null=0
        Delta_A=0
        Back2L= baseline_wheel2wheel
        Wheel2R=R*R
        Delta_x=0
        Delta_y=0


        while not rospy.is_shutdown():
            read = SMBus(1).read_byte_data(0x3e, 0x11)
            #Encoder Zeroing
            if (self.L_encoder>0 or self.R_encoder>0) and flag == 0:
                Display_L_en=self.L_encoder-Save_L_en
                Display_R_en=self.R_encoder-Save_R_en


            
            elif flag == 1 and (self.L_encoder != 0 or self.R_encoder!=0):
                Save_L_en = self.L_encoder
                Save_R_en = self.R_encoder

                flag = 0
                                                            #rataste encoderite nullimine

           
            L_Rotation= np.rad2deg(Display_L_en * ((2*np.pi)/N_tot))
            print(f"The left wheel rotated: {L_Rotation} degrees")
            R_Rotation= np.rad2deg(Display_R_en * ((2*np.pi)/N_tot))
            print(f"The right wheel rotated: {R_Rotation} degrees")
            L_Distance= R*L_Rotation
            print(f"The left wheel travel: {round(L_Distance,4)} cm")
            R_Distance= R*R_Rotation
            print(f"The righ wheel travel: {round(R_Distance,4)} cm")

            Delta_A=(R_Distance+L_Distance)/2
            print(f"Delta distance: {round(Delta_A,3)}")
            Delta_null= (R_Distance-L_Distance)/Back2L
            print(f"Delta rotation: {round(Delta_null,3)}")

            Delta_x=Delta_A*np.cos(Delta_null)
            print(f"Delta X: {round(Delta_x,3)}")
            Delta_y=Delta_A*np.sin(Delta_null)
            print(f"Delta Y: {round(Delta_y,3)}")

            #Distance= (L_Distance+R_Distance)/2
            #print(f"Distance average travel: {round(Distance ,3)} meters")

            if self.distance<0.3 and so==0:
                speed.vel_right=0
                speed.vel_left=0
                Save_R_deg= R_Rotation
                Save_L_deg= L_Rotation
                so=10

            #Keerab alguses
            if so == 10:

                speed.vel_right=0.17
                speed.vel_left=-0.17

                so=20

            if so == 20 and (Save_R_deg+90 < R_Rotation) and (Save_L_deg-90 > L_Rotation):
                speed.vel_right=0.0
                speed.vel_left=0.0
                
                Sec_save=self.sec
                
                so=30

            if so == 30 and (Sec_save+10<self.sec):

                speed.vel_right=0.2
                speed.vel_left=0.2
                Dst_Save=Delta_A
                so=40


            if so == 40 and (Dst_Save+3.0< Delta_A) and ( Dst_Save+3.0<Delta_A):
                speed.vel_right=0.0
                speed.vel_left=0.0
                Save_R_deg= R_Rotation
                Save_L_deg= L_Rotation                
                Sec_save=self.sec
                so=50
            if so == 50 and (Sec_save+10<self.sec):    
                speed.vel_right=-0.17
                speed.vel_left=0.17

                so=60

            if so == 60 and (Save_R_deg-90 < R_Rotation) and (Save_L_deg+90 > L_Rotation):
                speed.vel_right=0.0
                speed.vel_left=0.0
                Sec_save=self.sec
                so=70

            if so == 70 and (Sec_save+10<self.sec):

                speed.vel_right=0.2
                speed.vel_left=0.2
                
                Dst_Save=Delta_A
                so=80


            if so == 80 and (Dst_Save+2.0< Delta_A) and ( Dst_Save+2.0<Delta_A):
                speed.vel_right=0.0
                speed.vel_left=0.0
                Sec_save=self.sec

                so=90
            
            if so==90 and (Sec_save+10<self.sec):
                speed.vel_right=-0.17
                speed.vel_left=0.17
                Save_R_deg= R_Rotation
                Save_L_deg= L_Rotation
                so=100


            if so == 100 and (Save_R_deg-90 < R_Rotation) and (Save_L_deg+90 > L_Rotation):
                speed.vel_right=0.0
                speed.vel_left=0.0
                
                Sec_save=self.sec
                
                so=110

            if so == 110 and (Sec_save+10<self.sec):

                speed.vel_right=0.2
                speed.vel_left=0.2
                Dst_Save=Delta_A
                so=120


            if so == 120 and (Dst_Save+2.0< Delta_A) and ( Dst_Save+2.0<Delta_A):
                speed.vel_right=0.0
                speed.vel_left=0.0
                Sec_save=self.sec
                so=130

            if so == 130 and (Sec_save+10<self.sec):
                speed.vel_right=-0.17
                speed.vel_left=0.17
                Save_R_deg= R_Rotation
                Save_L_deg= L_Rotation
 
                so=140        



            if so == 140 and (Save_R_deg+90 < R_Rotation) and (Save_L_deg-90 > L_Rotation):
                speed.vel_right=0.0
                speed.vel_left=0.0
                
                Sec_save=self.sec
                
                so=150

            if so == 150 and (Sec_save+10<self.sec):

                speed.vel_right=0.0
                speed.vel_left=0.0
                Dst_Save=Delta_A
                so=0



            print("SO---"+(str(so)))
            print("Sec "+(str(self.sec)))
            print("Sec save "+(str(Sec_save)))
            #print("LAST R ENCODER: "+(str(Last_R_encoder)))
            #print("LAST L ENCODER: "+(str(N_tot)))
            print("DST Delta: "+(str(Delta_A)))
            print("Distance SAVE Delta: "+(str(Dst_Save)))
            print("DISPLAY R ENCODER: "+(str(Display_R_en)))
            print("DISPLAY L ENCODER: "+(str(Display_L_en)))
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