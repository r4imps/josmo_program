#!/usr/bin/env python3
import rospy

from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped
from sensor_msgs.msg import Range
from std_msgs.msg import String
from odometry import *



speed = WheelsCmdStamped()

class STRONK(DTROS):
    def __init__(self, node_name):
        super(STRONK, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        rospy.Subscriber('/josmo/front_center_tof_driver_node/range', Range, self.callback)
        self.distance=0
        self.pub = rospy.Publisher('/josmo/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        self.bits='0'
        rospy.Subscriber('/line_bits', String, self.line)
        rospy.set_param("/v_pid", [0.045 ,0.022 ,0.25 ,0.4])
        self.bits=0
        self.PID_Time_Last=0
        self.I=0
        self.error=0
        self.prev_int=0
        self.PID_DELTA=0
        self.prev_error=0
    
        

    def callback(self,data):
        self.distance= data.range
    def line(self,data):
        self.bits= data.data

    def on_shutdown(self):
        rospy.on_shutdown(self.shutdown)
    def shutdown(self):
        speed.vel_left = 0
        speed.vel_right = 0
        self.pub.publish(speed)
        rospy.on_shutdown()

    def run(self):
        rate = rospy.Rate(20)
        
        obj= ODOMEETRIA()
        while not rospy.is_shutdown():                
            
            
            def PID_STRT():
                
                Joonebitid=['10000000',
                '11000000',
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
                vel_to_right=0.0
                vel_to_left=0.0
                Kp,Ki,Kd,v_0 = rospy.get_param("/v_pid")
                PID_STRT=False
                    #==========LIST TO INDEX===================
                if self.bits in Joonebitid:
                    index = 12 - Joonebitid.index(self.bits)
                    print(f'index on: {index}')
                    PID_STRT=True
                #==========ROBOT PIT MODE=====================    
                elif self.bits == '11111111':
                    vel_to_right=0.0
                    vel_to_left=0.0
                    PID_STRT=False
                    #return [vel_to_right , vel_to_left]
                    
                #==============PID CONTROLLER==============================================================
                PID_Time= time.time()
               

                
                if PID_STRT ==True and PID_Time!=0 and self.PID_DELTA!=0 and self.bits!='':
                    #print("PID ACTIVE")
                    #KIIRUSE VÄHENDAMINE KUI ON LAI JOON
                    if self.bits=='00111100':
                        v_0=0.2

                    self.error= 6 - index
                    P= self.error
                    I=self.prev_int+(self.PID_DELTA*self.error)
                    I = max(min(I,1.0),-1.0)
                    D=((self.error-self.prev_error)/self.PID_DELTA)
                    PID= Kp*P+Ki*I+Kd*D
                    #print(f'SEE ON ERROR : {self.error},    PREVIOUS INT : {self.prev_int}')
                    #print(f'PID_START{PID_STRT}       TIME: {PID_Time} ,  LAST_TIME: {self.PID_Time_Last} ')
                    
                    vel_to_right=v_0+PID
                    vel_to_left=v_0-PID
                    if vel_to_left != None and vel_to_right!=None:
                        #return [vel_to_right , vel_to_left]
                        pass
                print(self.bits)
                self.PID_DELTA= PID_Time-self.PID_Time_Last
                self.PID_Time_Last=PID_Time
                self.prev_int=self.I
                self.prev_error=self.error

            
                
            
            self.pub.publish(speed)
            rate.sleep()
if __name__ == '__main__':
  node = STRONK(node_name='stronk_node')
  node.run()
  rospy.on_shutdown(STRONK.on_shutdown)
  rospy.spin()