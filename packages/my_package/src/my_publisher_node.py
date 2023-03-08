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
        rospy.Publisher('/josmo/line_reader/data', String, )

        rospy.Subscriber('/josmo/front_center_tof_driver_node/range', Range, self.callback)

        rospy.Subscriber('/josmo/right_wheel_encoder_node/tick', WheelEncoderStamped, self.Callback_R_Encoder)
        rospy.Subscriber('/josmo/left_wheel_encoder_node/tick', WheelEncoderStamped, self.Callback_L_Encoder)
        

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
        self.pub.publish(speed)
        rospy.on_shutdown()
    
      


    def run(self):

        #MUUTUJAD
        rate = rospy.Rate(50)
        flag=1
        flagtwo=0
        read_converted=0
        v_0= 0.5
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
        Delta_x=0
        Delta_y=0
        prev_bits=[]
#####################        
        prev_error=0
        prev_int=0
        error=0
        PID_Time_Last=self.sec+1
        PID_STRT=True
        P=0
        I=0
        D=0
        index=0
        tmp = []

#######################
        Joonebitid=[
        '11111000',
        '11110000',
        '11100000',
        '10000000',
        '11000000',
        '01000000',
        '01100000',
        '00100000',
        '00110000',
        '00010000',
        '00011000',
        '00001000',        
        '00001100',
        '00000100',
        '00000110',
        '00000010',
        '00000011',
        '00000001',
        '00000111',
        '00001111',
        '00011111']

        Suund = [
        '10011100',
        '00010010',
        '00011011',
        '00011001',
        '10001100',
        '00110110',
        '10110000',
        '10010000',
        '10111000',
        '10011000',
        '00001101',
        '00001001',
        '00110010',
        '00010011',
        '11011000',
        '00110001',
        '01100100',
        '00100100',
        '00011010',
        ]
        Haru=[
        '01100110',
        '00110110',
        '01100100',
        '11100110',
        '01100111',
        '01110110',
        '01101110',
        '01101100',
        '01001110',
        '11000110',
        '01100011',
        '11001110',
        '11101100',
        '11101000',
        '00010111',
        '00110111',
        '00100111', 
        '00010011',
        '00001001',
        '11001100',
        '00110011',
        '01000001',
        '10000010',
        '01001100',
        '00100110',
        '01100010'   
        ]
        
    

        while not rospy.is_shutdown():
            #==============LINE DETECTOR======================
            read = SMBus(1).read_byte_data(0x3e, 0x11)
            

            #================Encoder Zeroing===================
            if (self.L_encoder>0 or self.R_encoder>0) and flag == 0:
                Display_L_en=self.L_encoder-Save_L_en
                Display_R_en=self.R_encoder-Save_R_en
            elif flag == 1 and (self.L_encoder != 0 or self.R_encoder!=0):
                Save_L_en = self.L_encoder
                Save_R_en = self.R_encoder
                flag = 0
 
            #========LINE DETECTOR DEC TO BIN==========
            bits_block=bin(read)[2:]
            leading_zeros = 8 - len(bits_block)
            bits = leading_zeros*'0' + bits_block
            

            
            #==========PID PARAMEETRID================    
            Kp, Ki, Kd ,v_0 = rospy.get_param("/v_pid")
            #==========LIST TO INDEX===================
            if bits in Joonebitid:
                index = 20 - Joonebitid.index(bits)
                PID_STRT=True
            #==========ROBOT PIT MODE=====================    
            elif bits == '11111111':
                speed.vel_right=0
                speed.vel_left=0
                PID_STRT=False
            #==========TOF DETECTION=======================  
            if self.distance<0.3:
                speed.vel_right=0
                speed.vel_left=0
                PID_STRT=False
            #==========RISTMIKU VALIMINE====================
            if bits in Suund:
                #print('SUUNA VALIMINE -------------------------')
                flagtwo=1
                Sec_save=self.sec
            
            if flagtwo==1 and Sec_save+10 < self.sec:  
                v_0=0.2
                if (bits in Haru) or (bits in Suund):
                    speed.vel_right=0.1
                    speed.vel_left=-0.1
                    Sec_save=self.sec
                    PID_STRT=0
                #print(f"Sec save                        :  {str(Sec_save)}")
                #print(index)
                #print("Sec "+(str(self.sec)))
                if Sec_save+30 < self.sec:
                    flagtwo=0

#==============PID CONTROLLER==============================================================
            PID_Time= self.sec
            if PID_STRT ==True and (PID_Time>0) and PID_DELTA!=0:
                #KIIRUSE VÄHENDAMINE KUI ON LAI JOON
                if bits=='00111100':
                    v_0=0.2

                error= 10 - index
                P= error
                I=prev_int+(PID_DELTA*error)
                I = max(min(I,1.0),-1.0)
                D=((error-prev_error)/PID_DELTA)
                PID= Kp*P+Ki*I+Kd*D
                #print(f'SEE ON ERROR : {error},    PREVIOUS INT : {prev_int}')
                #print(f'PID:{PID},       TIME: {PID_Time} ,  LAST_TIME: {PID_Time_Last} ')
                speed.vel_right=v_0+PID
                speed.vel_left=v_0-PID
                #print(PID)
            
                #=========JOONELUGEJA VÄÄRTUSTE SALVESTAMINE===================
                if len(prev_bits)<=7:
                    prev_bits.append(bits)
                else:
                    prev_bits.pop(0)
                    prev_bits.append(bits)
                #print(f'RIGHT: {v_0+PID},    LEFT : {v_0-PID}')
            #print("=========================================================")
#============================================================================================     
            #============SALVESTATUD VÄÄRTUSTE VÄLJA PRINTIMINE
            print(f'PREVIOUS BIT :')
            for i in prev_bits:
                print(i)
                
            #=======PID PREVIOUS VALUES=================
            PID_DELTA= PID_Time-PID_Time_Last
            PID_Time_Last=PID_Time
            prev_int=I
            prev_error=error

           #==================ODOMEETRIA ARVUTUSKÄIK====================
            L_Rotation= Display_L_en * ((2*np.pi)/N_tot)
            #print(f"The left wheel rotated: {L_Rotation} degrees")

            R_Rotation= Display_R_en * ((2*np.pi)/N_tot)
            #print(f"The right wheel rotated: {R_Rotation} degrees")

            L_Distance= R*L_Rotation
            #print(f"The left wheel travel: {round(L_Distance,4)} m")

            R_Distance= R*R_Rotation
            #print(f"The righ wheel travel: {round(R_Distance,4)} m")

            Delta_A=(R_Distance+L_Distance)/2
            #print(f"Delta distance:===== {round(Delta_A,3)} =====")

            Delta_null= (R_Distance-L_Distance)/Back2L
            #print(f"Delta rotation: {round(Delta_null,3)}")

            Delta_x=Delta_A*np.cos(Delta_null)
            #print(f"Delta X:================= {round(Delta_x,2)} ================")

            Delta_y=Delta_A*np.sin(Delta_null)
            #print(f"Delta Y:================= {round(Delta_y,2)} ================")

            
            #==========================VÄÄRTUSTE VÄLJA PRINTIMINE================================
            #print("LAST R ENCODER: "+(str(Last_R_encoder)))
            #print("LAST L ENCODER: "+(str(N_tot)))
            #print("Save Left Degrees                 : "+(str(Save_L_deg)))
            #print("Save Right Degrees                : "+(str(Save_R_deg)))
            #print("Distance SAVE Delta                : "+((str(round(Dst_Save,3)))))
            #print("DISPLAY R ENCODER: "+(str(Display_R_en)))
            #print("DISPLAY L ENCODER: "+(str(Display_L_en)))
            #print(self.R_encoder)
            #print(self.L_encoder)
            #print("ToF Distance: "+(str(self.distance)))
            #print("Joon"+(str(read)))
            

            self.pub.publish(speed)

            rate = rospy.Rate(10)
            
            R_Distance=0
            L_Distance=0
            
            N_tot= 135
            R=0.03424
            baseline_wheel2wheel= 0.1


            while not rospy.is_shutdown():
                read = SMBus(1).read_byte_data(0x3e, 0x11)

                if self.L_encoder>0 and flag == 1:
                    Display_L_en=self.L_encoder-Save_L_en

                
                elif flag == 0 and self.L_encoder > 0:
                    Save_L_en = self.L_encoder
                    flag = 1
                                                                #rataste encoderite nullimine
                else:
                    continue

                if self.R_encoder>0 and flag == 1:
                    Display_R_en=self.R_encoder-Save_R_en

                
                elif flag == 0 and self.R_encoder > 0:
                    Save_R_en = self.R_encoder
                    flag = 1

                else:
                    continue
                
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



if __name__ == '__main__':
    node = ROSPROG(node_name='my_publisher_node')
    node.run()
    rospy.on_shutdown(ROSPROG.on_shutdown)
    rospy.spin()
