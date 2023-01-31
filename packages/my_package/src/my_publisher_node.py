#!/usr/bin/env python3
import os
import rospy
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
        T_Start=0
        T_Done=0    
        Saved_Time=0.0
        x=0
   

        while not rospy.is_shutdown():
            read = SMBus(1).read_byte_data(0x3e, 0x11)
            Timer= time.time()
            

            # encoder value
            
            if self.L_encoder!=Save_L_en:
                Display_L_en=Display_L_en+1
            Save_L_en=self.L_encoder    

            if self.R_encoder!=Save_R_en:
                Display_R_en=Display_R_en+1
            Save_R_en=self.R_encoder
                
            if T_Start==1:
                Time_set=3.0
                x=Saved_Time+Time_set
                if Timer == x:
                    T_Done=1




            if self.distance<0.3 and so==0:
                speed.vel_right=0
                speed.vel_left=0

                so=10

            #Keerab alguses
            if so == 10 :
                Last_R_encoder= Display_R_en
                Last_L_encoder= Display_L_en
                
                so=20

            if so==20:
                speed.vel_right=0.14
                speed.vel_left=-0.14
                if (Display_R_en>(Last_R_encoder+3)) and (Display_L_en>(Last_L_encoder+3)) :
                    speed.vel_right=0
                    speed.vel_left=0

                    so=30                
           #Otse
            if so==30:
                Last_R_encoder= Display_R_en
                Last_L_encoder= Display_L_en
                Saved_Time= Timer
                T_Start=1
                so=40
                    
            if so==40 and T_Done==1:
                speed.vel_right=0.3
                speed.vel_left=0.3
                T_Start=0

                if  (Display_R_en>(Last_R_encoder+10)) and (Display_L_en>(Last_L_encoder +10)):
                    speed.vel_right=0
                    speed.vel_left=0
                    
                    so=50
            #Keerab paremale
            if so == 50:
                Last_R_encoder= Display_R_en
                Last_L_encoder= Display_L_en
                
                so=60

            if so==60:
                speed.vel_right=-0.2
                speed.vel_left=0.2
                if (Display_R_en>(Last_R_encoder+2)) and (Display_L_en>(Last_L_encoder+2)) :
                    speed.vel_right=0
                    speed.vel_left=0
                    so=70
            #Otse          
            if so==70:
                Last_R_encoder= Display_R_en
                Last_L_encoder= Display_L_en
                
                so=80
                    
            if so==80:
                speed.vel_right=0.3
                speed.vel_left=0.3

                if  (Display_R_en>(Last_R_encoder+10)) and (Display_L_en>(Last_L_encoder+10)):
                    speed.vel_right=0
                    speed.vel_left=0
                    
                    so=90 
            #Keerab paremale                                              
            if so == 90:
                Last_R_encoder= Display_R_en
                Last_L_encoder= Display_L_en
                
                so=100

            if so==100:
                speed.vel_right=-0.2
                speed.vel_left=0.2
                if (Display_R_en>(Last_R_encoder+2)) and (Display_L_en>(Last_L_encoder+2)) :
                    speed.vel_right=0
                    speed.vel_left=0
                    so=0                            
                    


      
                            
                
            

            print("SO---"+(str(so)))
      
            print("now SECS: "+ (str(Timer)))
            print("done SECS: "+ (str(T_Done)))
            print("done SECS: "+ (str(T_Done)))
            print("saved SECS: "+ (str(x)))
            print("Start SECS: "+ (str(T_Start)))

            #print("LAST R ENCODER: "+(str(Last_R_encoder)))
            #print("LAST L ENCODER: "+(str(Last_L_encoder)))
            #print("SAVE R: "+(str(Save_R_en)))
            #print("SAVE L: "+(str(Save_L_en)))
            print("DISPLAY L ENCODER: "+(str(Display_L_en)))
            print("DISPLAY R ENCODER: "+(str(Display_R_en)))
            #print(self.R_encoder)
            #print(self.L_encoder)
            print("ToF Distance: "+(str(self.distance)))
            #print("Joon"+(str(read)))
            self.pub.publish(speed)

            rate.sleep()

if __name__ == '__main__':
    node = ROSPROG(node_name='my_publisher_node')
    node.run()
    rospy.on_shutdown(ROSPROG.on_shutdown)
    rospy.spin()
