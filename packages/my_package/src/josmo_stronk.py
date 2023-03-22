#!/usr/bin/env python3
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped
from sensor_msgs.msg import Range
from std_msgs.msg import String
from PID_Controller import *
from odometry import *



speed = WheelsCmdStamped()

class STRONK(DTROS):
    def __init__(self, node_name):
        super(STRONK, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        rospy.Subscriber('/josmo/front_center_tof_driver_node/range', Range, self.callback)
        self.distance=0
        self.pub = rospy.Publisher('/josmo/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        self.bits='0'
    
        

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
        obj_pid=PID()
        obj= ODOMEETRIA()
        while not rospy.is_shutdown():
            
            """Val= obj_pid.PID_STRT()
            if Val!=None:
                speed.vel_right= Val[0]
                speed.vel_left= Val[1]"""
                
                
            
            VALUES=obj.ODOMETRY_FUNC()
            print(VALUES[6], VALUES[7])
            """if VALUES[6]<2 and VALUES[6]>1:
               speed.vel_right=0.4
               speed.vel_left= 0.4
            elif VALUES[6]>2:
                speed.vel_right=0.1
                speed.vel_left= 0.3
            elif VALUES[6]<1:
                speed.vel_right=0.3
                speed.vel_left= 0.1"""
            
            self.pub.publish(speed)
            rate.sleep()
if __name__ == '__main__':
  node = STRONK(node_name='stronk_node')
  node.run()
  rospy.on_shutdown(STRONK.on_shutdown)
  rospy.spin()