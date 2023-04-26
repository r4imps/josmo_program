#!/usr/bin/env python3
import rospy
from duckietown.dtros import DTROS, NodeType

from sensor_msgs.msg import Imu 
from std_msgs.msg import String
import time
from math import sin



class IMU_Prog(DTROS):
    def __init__(self, node_name):
        super(IMU_Prog, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        rospy.Subscriber('/josmo/imu_node/imu_data', Imu, self.callback)
     

        self.angular_velocity_x = 0.0
        self.angular_velocity_y = 0.0
        self.angular_velocity_z = 0.0
        self.linear_acceleration_x = 0.0
        self.linear_acceleration_y = 0.0
        self.linear_acceleration_z = 0.0
        self.Imu_parameters=[0.0,0.0,0.0]

 

        





    def callback(self,data):
        self.angular_velocity_x = data.angular_velocity.x
        self.angular_velocity_y = data.angular_velocity.y
        self.angular_velocity_z = data.angular_velocity.z
        self.linear_acceleration_x = data.linear_acceleration.x
        self.linear_acceleration_y = data.linear_acceleration.y
        self.linear_acceleration_z = data.linear_acceleration.z


    def run(self):
        x=0
        vx=0
        th=0
        while not rospy.is_shutdown():
            #print("---------------------------------------------------")
            #print([round(self.angular_velocity_x,3), self.angular_velocity_y, self.angular_velocity_z])
            #print([round(self.linear_acceleration_y/self.gravity,1), round(-self.linear_acceleration_x/self.gravity,1)])
            ct = rospy.Time.now()
            lt=ct
            dt=ct-lt
            #
            rotz=self.angular_velocity_z
            dth=rotz*dt
            th=th+dth

            #
            accx=self.linear_acceleration_x
            dvx=accx*dt
            vx=vx+dvx
            dx=vx*dt
            x=x+(dx*sin(th))
            #print("Current position: ({:.2f}, {:.2f})".format(current_pos[0], current_pos[1]))
            print(x)
            time.sleep(0.1)
            

if __name__ == '__main__':
  node = IMU_Prog(node_name='imu_node')
  node.run()
  rospy.spin()