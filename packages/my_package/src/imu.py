#!/usr/bin/env python3
import rospy
from duckietown.dtros import DTROS, NodeType

from sensor_msgs.msg import Imu 
from std_msgs.msg import String
import time




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

        self.save=0
        self.gravity=9.81

        self.last_time = rospy.Time.now()
        self.last_vel = [0, 0]
        self.last_pos = [0, 0]
        self.orientation = [0, 0, 0, 0]





    def callback(self,data):
        self.angular_velocity_x = data.angular_velocity.x
        self.angular_velocity_y = data.angular_velocity.y
        self.angular_velocity_z = data.angular_velocity.z
        self.linear_acceleration_x = data.linear_acceleration.x
        self.linear_acceleration_y = data.linear_acceleration.y
        self.linear_acceleration_z = data.linear_acceleration.z


    def run(self):
       
        while not rospy.is_shutdown():
            #print("---------------------------------------------------")
            #print([round(self.angular_velocity_x,3), self.angular_velocity_y, self.angular_velocity_z])
            print([round(self.linear_acceleration_y/self.gravity,1), round(-self.linear_acceleration_x/self.gravity,1)])
            current_time = rospy.Time.now()
            delta_time = (current_time - self.last_time).to_sec()
            self.last_time = current_time

            
            current_vel = [
            self.last_vel[0] + self.linear_acceleration_y * delta_time,
            self.last_vel[1] - self.linear_acceleration_x * delta_time ]
            self.last_vel = current_vel

        # Compute the robot's position by integrating the velocity
            current_pos = [
            self.last_pos[0] + current_vel[0] * delta_time,
            self.last_pos[1] + current_vel[1] * delta_time]
            self.last_pos = current_pos

            print("Current position: ({:.2f}, {:.2f})".format(current_pos[0], current_pos[1]))
            time.sleep(0.1)
            

if __name__ == '__main__':
  node = IMU_Prog(node_name='imu_node')
  node.run()
  rospy.spin()