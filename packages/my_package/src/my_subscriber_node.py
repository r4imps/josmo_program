#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String

from sensor_msgs.msg import Range

from smbus2 import SMBus


class MySubscriberNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MySubscriberNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        self.sub = rospy.Subscriber('sparkfun_line_array', String, self.callback)


    def run(self, data):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            lanereader_value = SMBus(1).read_byte(0x3e, 0x11)

            rospy.loginfo(f"Lanereader: {lanereader_value}"
            self.sub.publish(str(lanereader_value))



if __name__ == '__main__':
    # create the node
    node = MySubscriberNode(node_name='my_subscriber_node')
    # keep spinning
    rospy.spin()
