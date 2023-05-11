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
        rate = rospy.Rate(20)
        prev_bits = []
        last_time = time.time() - 0.002
        prev_e = 0.0
        prev_int = 0.0
        start_time = last_time

        avoiding_obstruction = False

        while not rospy.is_shutdown():
            while self.distance == 0.0:
                rate.sleep()
                        
        #roboti käivitamisel peab pisut ootama kuni kõik sensorid käivitatakse,
        #vastasel juhul saab programm vale andmeid ja hakkab nende põhjal tegema mitte vajalike operatsioone

######################################      Move             ###################################

            if 0.5 < self.distance < 10.0 and self.bits != '11111111' and avoiding_obstruction == False:
                delta_t = time.time()- last_time
                v_0, omega, prev_e, prev_int = PIDController(self.bits, prev_e, prev_int, delta_t, prev_bits)
                speed.vel_right = v_0 + omega
                speed.vel_left = v_0 - omega
                self.pub.publish(speed)

                start_time = time.time()  #takistuse tuvastamise aeg
            
            elif self.distance < 0.5 or avoiding_obstruction == True:
                speed.vel_right, speed.vel_left , avoiding_obstruction = AvoidObstacle(start_time)
                self.pub.publish(speed)

            else:          ############### PIT MODE ####################
                speed.vel_right = 0.0
                speed.vel_left = 0.0
                self.pub.publish(speed)
                delta_t = 0.0
                print(f'PIT MODE')

                            ############### 8X8 log ################

            if len(prev_bits)<=7:
                prev_bits.append(self.bits)
            else:
                prev_bits.pop(0)
                prev_bits.append(self.bits)

            print(f'vel_right: {speed.vel_right} vel_left: {speed.vel_left} Bits: {self.bits} distance: {self.distance}')
            
            rate.sleep()
            last_time = time.time()
 
            


if __name__ == '__main__':
    node = STRONK(node_name='stronk_node')
    node.run()
    rospy.on_shutdown(STRONK.on_shutdown)
    rospy.spin()