#!/usr/bin/env python3
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import Range



class TOF_prg(DTROS):
  def __init__(self, node_name):

    # initialize the DTROS parent class
    super(TOF_prg, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
    rospy.Subscriber('/josmo/front_center_tof_driver_node/range', Range, self.callback)
    self.distance=0
  def callback(self,data):
    self.distance= data.range
  
  def Go_Around(self):
    pass
        



if __name__ == '__main__':
  node = TOF_prg(node_name='ob_detect_node')
  node.Go_Around()
  rospy.spin()