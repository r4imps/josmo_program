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

        self.distance = 0.0
        self.bits = ''
    
    def callback(self, data) -> str:
        self.bits = data.data
    
    def callback_ToF(self, data) -> float:
        self.distance = data.range

    def on_shutdown(self):
        speed.vel_left = 0.0
        speed.vel_right = 0.0
        self.pub.publish(speed)
        rospy.on_shutdown()

    def run(self):
        rate = rospy.Rate(1000)
        prev_bits = []
        last_time = time.time() - 0.002
        prev_e = 0.0
        prev_int = 0.0

        flag = False

        while not rospy.is_shutdown():

######################################      Move             ###################################

            if 0.4 < self.distance < 10.0 and self.bits != '11111111':
                delta_t = time.time()- last_time
                v_0, omega, prev_e, prev_int = PIDController(self.bits, prev_e, prev_int, delta_t, prev_bits)
                speed.vel_right = v_0 + omega
                speed.vel_left = v_0 - omega
                self.pub.publish(speed)
                flag = False
                                
                            ############### 8X8 log ################

                if len(prev_bits)<=7:
                    prev_bits.append(self.bits)
                else:
                    prev_bits.pop(0)
                    prev_bits.append(self.bits)
                
                for i in prev_bits:
                    #print(i)
                    pass

            else:          ############### PIT MODE ####################
                speed.vel_right = 0.0
                speed.vel_left = 0.0
                self.pub.publish(speed)
                delta_t = 0.0
                if flag == False:
                    print(f'PIT MODE')
                    flag = True

                
            
            rate.sleep()
            last_time = time.time()
            #print(f'last_time = {last_time}    delta_t = {delta_t}')    
            


if __name__ == '__main__':
    node = STRONK(node_name='stronk_node')
    node.run()
    rospy.on_shutdown(STRONK.on_shutdown)
    rospy.spin()