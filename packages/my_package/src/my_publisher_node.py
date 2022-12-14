#!/usr/bin/env python3

import os
import rospy
import time 
from duckietown.dtros import DTROS, NodeType, TopicType
from std_msgs.msg import String
from smbus2 import SMBus
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped

class MyLine_followerNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyLine_followerNode, self).__init__(node_name=MyLine_followerNode, node_type=NodeType.GENERIC)
        # construct publisher
        self.pub = rospy.Publisher('sparkfun_line_follower', queue_size=10)
        # Setup publishers
        
    

    def run(self):
        # publish message every 1 second
        rate = rospy.Rate(20)
        msg_wheels_cmd = WheelsCmdStamped()
        while not rospy.is_shutdown():

            self.lanereader_value = SMBus(1).read_byte_data(0x3e, 0x11)
            rospy.loginfo(f"Lanereader: '{self.lanereader_value}'")
            self.pub.publish(str(self.lanereader_value))
             

            rate.sleep()

class MyWheelsNode(DTROS):

    def __init__(self) -> None:
        super(MyWheelsNode,self).__init__(node, NodeType = NodeType.CONTROL)

        self.pub = rospy.Publisher()


if __name__ == '__main__':
    # create the node
    node = MyLine_followerNode(node_name='MyLine_followerNode')
    # run node
    node.run()
    # keep spinning
    rospy.spin()
