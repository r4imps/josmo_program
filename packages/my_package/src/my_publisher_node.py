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





speed = WheelsCmdStamped()

class ROSPROG(DTROS):
    def __init__(self, node_name):

        # initialize the DTROS parent class
        super(ROSPROG, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # SUBSCRIBERID JA PUBLISHERID
        self.pub = rospy.Publisher('/josmo/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
       
        self.distance=0

        
        



    def on_shutdown(self):
        rospy.on_shutdown(self.shutdown)
    def shutdown(self):
        speed.vel_left = 0
        speed.vel_right = 0
        self.pub.publish(speed)

    
    def run(self):

        while not rospy.is_shutdown():
            
            self.pub.publish(speed)
            rate = rospy.Rate(10)

if __name__ == '__main__':
    node = ROSPROG(node_name='my_publisher_node')
    node.run()
    rospy.spin()
    rospy.on_shutdown(ROSPROG.on_shutdown)
    
