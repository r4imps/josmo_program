#!/usr/bin/env python3
import rospy

from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped
from sensor_msgs.msg import Range
from std_msgs.msg import String
from odometry import *
import PID_Controller 






class STRONK(DTROS):
    def __init__(self, node_name):
        super(STRONK, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        rospy.Subscriber('/josmo/front_center_tof_driver_node/range', Range, self.callback)
        self.pub = rospy.Publisher('/josmo/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        

        self.distance=0

    def callback(self,data):
        self.distance= data.range

    def on_shutdown(self):
        rospy.on_shutdown(self.shutdown)
    def shutdown(self):
        speed.vel_left = 0
        speed.vel_right = 0
        self.pub.publish(speed)
        rospy.on_shutdown()

    def run(self):
        t0=time.time()
        v_0=0.2
        rate = rospy.Rate(20)
        
        while not rospy.is_shutdown():
            #PID#
            t1=time.time()
            
            speed.vel_right=v_0-PID_Controller.pid_controller(t0,t1)
            speed.vel_left=v_0+PID_Controller.pid_controller(t0,t1)
            

            if self.distance<0.25:
                self.ob_avoid()
                print("OB")
            
            self.pub.publish(speed)    
            
            rate.sleep()
    def ob_avoid(self):
        
        speed.vel_right=0
        speed.vel_left=0
        self.pub.publish(speed)
        rospy.sleep(1)

        speed.vel_right=0.5
        speed.vel_left=0
        self.pub.publish(speed)
        rospy.sleep(0.45)

        speed.vel_right=0.1
        speed.vel_left=0.25
        self.pub.publish(speed)
        rospy.sleep(1.5)


        speed.vel_right=0
        speed.vel_left=0.25
        self.pub.publish(speed)
        rospy.sleep(0.5)

        speed.vel_right=0.2
        speed.vel_left=0.2
        self.pub.publish(speed)
        rospy.sleep(1.5)

        speed.vel_right=0.0
        speed.vel_left=0.2
        self.pub.publish(speed)
        rospy.sleep(1)

        speed.vel_right=0.2
        speed.vel_left=0.2
        self.pub.publish(speed)
        rospy.sleep(1)

        speed.vel_right=0.2
        speed.vel_left=0.1
        self.pub.publish(speed)
        rospy.sleep(0.3)

if __name__ == '__main__':
  speed = WheelsCmdStamped()
  node = STRONK(node_name='stronk_node')
  node.run()
  rospy.on_shutdown(STRONK.on_shutdown)
  rospy.spin()