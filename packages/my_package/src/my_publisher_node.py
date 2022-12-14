#!/usr/bin/env python3

import os
import rospy
import time 
from duckietown.dtros import DTROS, NodeType, TopicType
from std_msgs.msg import String
from smbus2 import SMBus
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped

class MyPublisherNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        self.pub = rospy.Publisher('josmo/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        # Setup publishers
        
    

    def run(self):
        # publish message every 1 second
        rate = rospy.Rate(20)
        msg_wheels_cmd = WheelsCmdStamped()
        while not rospy.is_shutdown():

            self.lanereader_value = SMBus(1).read_byte_data(0x3e, 0x11)
            #rospy.loginfo(f"Lanereader: '{self.lanereader_value}'")
            #self.pub.publish(str(self.lanereader_value))
             


    

            msg_wheels_cmd.vel_right = 1.0
            msg_wheels_cmd.vel_left = 1.0
            time.sleep(2)
            msg_wheels_cmd.vel_right = 0
            msg_wheels_cmd.vel_left = 0

            self.pub.publish(msg_wheels_cmd)
            

            rate.sleep()


if __name__ == '__main__':
    # create the node
    node = MyPublisherNode(node_name='my_publisher_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()
