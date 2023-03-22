#!/usr/bin/env python3
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import Imu
from std_msgs.msg import String



class IMU_Prog(DTROS):
    def __init__(self, node_name):
        super(IMU_Prog, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        rospy.Subscriber('/josmo/imu_node/imu_data', Imu, self.callback)
        self.Imu_parameters=[0.0,0.0,0.0]
        self.orientation=[]
    def callback(self,data):
        self.orientation= data.orientation_covariance

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            
            rate.sleep()

if __name__ == '__main__':
  node = IMU_Prog(node_name='imu_node')
  node.run()
  rospy.spin()