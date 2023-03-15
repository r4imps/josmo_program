#!/usr/bin/env python3
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped
from sensor_msgs.msg import Range
from std_msgs.msg import String
from PID_Controller import *



speed = WheelsCmdStamped()

class STRONK(DTROS):
    def __init__(self, node_name):
        super(STRONK, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        rospy.Subscriber('/josmo/front_center_tof_driver_node/range', Range, self.callback)
        self.distance=0
        self.pub = rospy.Publisher('/josmo/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        rospy.Subscriber('/line_bits', String, self.line)
        self.bits='0'

        

    def callback(self,data):
        self.distance= data.range
    def line(self,data):
        self.bits= data.data

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            print(PID_STRT(self.bits))
            self.pub.publish(speed)
            rate.sleep()

if __name__ == '__main__':
  node = STRONK(node_name='stronk_node')
  node.run()
  rospy.spin()