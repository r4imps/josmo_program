#!/usr/bin/env python3
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
import rospy
from smbus2 import SMBus
from Biti_Vabriks import *

class ROSPROG(DTROS):
    def __init__(self, node_name):
        super(ROSPROG, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        self.pub = rospy.Publisher('/josmo/line_reader/data', String)

    def run(self):
        while not rospy.is_shutdown():
            #==============LINE DETECTOR======================
            read = SMBus(1).read_byte_data(0x3e, 0x11)
            bits_block=bin(read)[2:]
            leading_zeros = 8 - len(bits_block)
            bits = leading_zeros*'0' + bits_block
            self.pub.publish(bits)

if __name__ == '__main__':
    node = ROSPROG(node_name= 'line_reader')
    node.run()
    rospy.spin()