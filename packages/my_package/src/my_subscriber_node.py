#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType

from duckietown_msgs.msg import WheelEncoderStamped
from sensor_msgs.msg import Range
from smbus2 import SMBus


class MySubscriberNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MySubscriberNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        
        rospy.Subscriber('/josmo/right_wheel_encoder_node/tick', WheelEncoderStamped, self.Callback_R_Encoder)
        rospy.Subscriber('/josmo/left_wheel_encoder_node/tick', WheelEncoderStamped, self.Callback_L_Encoder)
        # construct publisher
        rospy.Subscriber('/josmo/front_center_tof_driver_node/range', Range, self.callback)

        rospy.Subscriber('/josmo/right_wheel_encoder_node/tick', WheelEncoderStamped, self.Callback_R_Encoder)
        rospy.Subscriber('/josmo/left_wheel_encoder_node/tick', WheelEncoderStamped, self.Callback_L_Encoder)

        self.distance=1.0
        self.R_encoder=0
        self.L_encoder=0
        self.sec=0

        

    def run(self, data):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            read = SMBus(1).read_byte(0x3e, 0x11)

            bits_block=bin(read)[2:]
            leading_zeros = 8 - len(bits_block)
            bits = leading_zeros*'0' + bits_block
            self.sub.publish(str(bits))



if __name__ == '__main__':
    # create the node
    node = MySubscriberNode(node_name='my_subscriber_node')
    
    # keep spinning
    rospy.spin()
