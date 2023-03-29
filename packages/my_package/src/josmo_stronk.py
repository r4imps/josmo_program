#!/usr/bin/env python3
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import String
from sensor_msgs.msg import Range
from PID_Controller import PIDController
import time
from AvoidObstacle import AvoidObstacle

speed = WheelsCmdStamped()

class STRONK(DTROS):
    def __init__(self, node_name):
        super(STRONK, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.pub = rospy.Publisher('/josmo/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        rospy.Subscriber('/josmo/line_reader/data', String, self.callback)
        rospy.Subscriber('/josmo/front_center_tof_driver_node/range', Range, self.callback_ToF)

        self.prev_bits = []
        self.sec = 0.1
        self.last_time = 0.0
        self.delta_t = 0.0

        self.prev_e = 0.0
        self.prev_int = 0.0

        self.distance = 0.0
        self.bits = ""

    
    def callback(self, data):
        self.bits = data.data
    
    def callback_ToF(self, data):
        self.distance = data.range

    def on_shutdown(self):
        speed.vel_left = 0.0
        speed.vel_right = 0.0
        self.pub.publish(speed)
        rospy.on_shutdown()

    def run(self):
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():

            self.sec = time.time()

######################################      Obstruction      ###################################

            if self.distance < 0.4:
                obstruction = True
                speed.vel_right = 0.0
                speed.vel_left = 0.0
                self.pub.publish(speed)

            elif self.bits == "11111111":
                obstruction = None
            else:
                obstruction = False

######################################      Move             ###################################

            if obstruction == False:
                self.delta_t = self.sec - self.last_time
                v_0, omega, self.prev_e, self.prev_int = PIDController(self.bits, self.prev_e, self.prev_int, self.delta_t)
                speed.vel_right = v_0 + omega
                speed.vel_left = v_0 - omega
                self.pub.publish(speed)

            elif obstruction == True:
                print(f'Obstruction')
                AvoidObstacle(self.distance)
                
            else:
                speed.vel_right = 0.0
                speed.vel_left = 0.0
                self.pub.publish(speed)
                print(f'PIT MODE')
            
            rate.sleep()

            self.last_time = self.sec
            
                            ############### 8X8 log ################

            if len(self.prev_bits)<=7:
                self.prev_bits.append(self.bits)
            else:
                self.prev_bits.pop(0)
                self.prev_bits.append(self.bits)

            print(f'PREVIOUS BITS :         Delta_t:  {self.delta_t}         Time:{self.sec}  Last_time:{self.last_time}')
            
            for i in self.prev_bits:
                print(i)

if __name__ == '__main__':
    node = STRONK(node_name='stronk_node')
    node.run()
    rospy.on_shutdown(STRONK.on_shutdown)
    rospy.spin()