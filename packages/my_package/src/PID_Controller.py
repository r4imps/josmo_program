#!/usr/bin/env python3
import time
import rospy
from std_msgs.msg import String


I=0
error=0
prev_int=0
PID_Time_Last=0
class PID():
    def __init__(self):
        rospy.Subscriber('/line_bits', String, self.line)
        rospy.set_param("/v_pid", [0.045 ,0.022 ,0.25 ,0.4])
        self.bits=0
    def line(self,data):
        self.bits= data.data
        
    def PID_STRT(self):
        
        Joonebitid=['10000000',
        '11000000',I=0
error=0
prev_int=0
        '00110000',
        '00010000',
        '00011000',
        '00001000',
        '00001100',
        '00000100',
        '00000110',
        '00000010',
        '00000011',
        '00000001']
        

    
    #==========PID PARAMEETRID================
    # 
        global I
        global prev_error
        global error
        global prev_int
        global PID_DELTA
        global PID_Time_Last
        vel_to_right=0.0
        vel_to_left=0.0
        
         
        Kp,Ki,Kd,v_0 = rospy.get_param("/v_pid")
        PID_STRT=False

            #==========LIST TO INDEX===================
        if self.bits in Joonebitid:
            index = 14 - Joonebitid.index(self.bits)
            #print(f'index on: {index}')
            PID_STRT=True
        #==========ROBOT PIT MODE=====================    
        elif self.bits == '11111111':
            vel_to_right=0.0
            vel_to_left=0.0
            PID_STRT=False
            return [vel_to_right , vel_to_left]
            
        #==============PID CONTROLLER==============================================================
        PID_Time= time.time()
         
        if PID_STRT ==True and PID_Time!=0 and PID_DELTA!=0 and self.bits:
            #print("PID ACTIVE")
            #KIIRUSE VÄHENDAMINE KUI ON LAI JOON
            if self.bits=='00111100':
                v_0=0.2

            error= 7 - index
            P= error
            I=prev_int+(PID_DELTA*error)
            I = max(min(I,1.0),-1.0)
            D=((error-prev_error)/PID_DELTA)
            PID= Kp*P+Ki*I+Kd*D
            #print(f'SEE ON ERROR : {error},    PREVIOUS INT : {prev_int}')
            #print(f'PID_START{PID_STRT}       TIME: {PID_Time} ,  LAST_TIME: {PID_Time_Last} ')
            vel_to_right=v_0+PID
            vel_to_left=v_0-PID
            if vel_to_left != None and vel_to_right!=None:
                return [vel_to_right , vel_to_left]
        PID_DELTA= PID_Time-PID_Time_Last
        PID_Time_Last=PID_Time
        prev_int=I
        prev_error=error
        