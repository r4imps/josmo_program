#!/usr/bin/env python3
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from smbus2 import SMBus

class Line_read(DTROS):
  def __init__(self, node_name):
    super(Line_read, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
    self.pub= rospy.Publisher('/line_bits', String, queue_size=10)
    self.bus= SMBus(1)
  def run(self):
      rate = rospy.Rate(20)
      while not rospy.is_shutdown():
          read = self.bus.read_byte_data(0x3e, 0x11)
          bits_block=bin(read)[2:]
          leading_zeros = 8 - len(bits_block)
          bits = leading_zeros*'0' + bits_block
          
          self.pub.publish(str(bits))
          rate.sleep()
          
if __name__ == '__main__':
  node = Line_read(node_name='line_value_node')
  node.run()
  rospy.spin()