#!/usr/bin/env python3
import rospy

from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped
from duckietown_msgs.msg import WheelEncoderStamped
from sensor_msgs.msg import Range
from std_msgs.msg import String
from odometry import *
import PID_Controller

speed = WheelsCmdStamped()

class STRONK(DTROS):
    def __init__(self, node_name):
        super(STRONK, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        rospy.Subscriber('/josmo/front_center_tof_driver_node/range', Range, self.callback)
        self.pub = rospy.Publisher('/josmo/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        rospy.Subscriber('/josmo/left_wheel_encoder_node/tick', WheelEncoderStamped, self.LeftEncoder)
        rospy.Subscriber('/josmo/right_wheel_encoder_node/tick', WheelEncoderStamped, self.RightEncoder)
        
        self.ticks_right=0
        self.ticks_left=0
        self.distance=0
        self.right=[[1,2,3],[1,2,3,4],[1,2,3,4,5],[2,3,4]]
        self.left=[[6,7,8],[5,6,7,8],[4,5,6,7,8],[7,6,5]]

    def callback(self,data):
        self.distance= data.range

    #LEFT&&RIGHT ENCODERs    
    def LeftEncoder(self,data):
        self.ticks_left = data.data    
    def RightEncoder(self,data):
        self.ticks_right=data.data

    def on_shutdown(self):
        rospy.on_shutdown(self.shutdown)
    def shutdown(self):
        speed.vel_left = 0
        speed.vel_right = 0
        self.pub.publish(speed)
        rospy.on_shutdown()

    def run(self):
        t0=time.time()
        v_0=0.2
        rate = rospy.Rate(20)
        flag=2
        while not rospy.is_shutdown():
            #PID#
            t1=time.time()
            
            
            
            
            if PID_Controller.get_line_values() in self.right:
                flag = 1
                

            if PID_Controller.get_line_values() in self.left:
                flag = 0
                
            
            while PID_Controller.get_line_values() == [] and flag == 1:
                speed.vel_left = 0.2
                speed.vel_right = -0.1
                self.pub.publish(speed)
                
            while PID_Controller.get_line_values() == [] and flag == 0:
                speed.vel_left = -0.1
                speed.vel_right = 0.2
                self.pub.publish(speed)
            
            speed.vel_right=v_0-PID_Controller.pid_controller(t0,t1)
            speed.vel_left=v_0+PID_Controller.pid_controller(t0,t1)
            print(PID_Controller.get_line_values())
           
            
            
            
            if self.distance<0.25:
                self.ob_avoid()
                print("OB")
            
            self.pub.publish(speed)    
            rate.sleep()

    def ob_avoid(self):
        speed.vel_right=0
        speed.vel_left=0
        self.pub.publish(speed)
        rospy.sleep(1)

        speed.vel_right=0.5
        speed.vel_left=0
        self.pub.publish(speed)
        rospy.sleep(0.45)

        speed.vel_right=0.1
        speed.vel_left=0.25
        self.pub.publish(speed)
        rospy.sleep(1.5)


        speed.vel_right=0
        speed.vel_left=0.25
        self.pub.publish(speed)
        rospy.sleep(0.5)

        speed.vel_right=0.2
        speed.vel_left=0.2
        self.pub.publish(speed)
        rospy.sleep(1.5)

        speed.vel_right=0.0
        speed.vel_left=0.2
        self.pub.publish(speed)
        rospy.sleep(1)

        speed.vel_right=0.2
        speed.vel_left=0.2
        self.pub.publish(speed)
        rospy.sleep(1)

        speed.vel_right=0.2
        speed.vel_left=0.1
        self.pub.publish(speed)
        rospy.sleep(0.3)

if __name__ == '__main__':
  node = STRONK(node_name='stronk_node')
  node.run()
  rospy.on_shutdown(STRONK.on_shutdown)
  rospy.spin()