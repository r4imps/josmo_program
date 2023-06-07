#!/usr/bin/env python3
import rospy
import numpy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import String
from sensor_msgs.msg import Range
import PID_Controller
import time
from Biti_Vabriks import *

speed = WheelsCmdStamped()

class STRONK(DTROS):
    def __init__(self, node_name):
        super(STRONK, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.pub = rospy.Publisher('/josmo/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        rospy.Subscriber('/josmo/line_reader/data', String, self.callback)
        rospy.Subscriber('/josmo/front_center_tof_driver_node/range', Range, self.callback_ToF)

        self.distance = 0.0
        self.bits = ''
        self.average_distances = []
        self.prev_bits = ['00011000']
    
    def callback(self, data) -> str:
        self.bits = data.data
    
    def callback_ToF(self, data) -> float:
        self.distance = data.range

    def on_shutdown(self):
        speed.vel_left = 0.0
        speed.vel_right = 0.0
        self.pub.publish(speed)
        rospy.on_shutdown()

    def ob_avoid(self):
        
        speed.vel_right=0
        speed.vel_left=0
        self.pub.publish(speed)
        rospy.sleep(0.25)

        speed.vel_right=0
        speed.vel_left=0.35
        self.pub.publish(speed)
        rospy.sleep(0.65)

        speed.vel_right=0.47
        speed.vel_left=0.16
        self.pub.publish(speed)
        rospy.sleep(3.4)

        speed.vel_right=0.2
        speed.vel_left=0.3
        self.pub.publish(speed)
        while self.bits == '00000000':
            rospy.sleep(0.05)

        speed.vel_right=0
        speed.vel_left=0
        self.pub.publish(speed)
        rospy.sleep(0.25)

        speed.vel_right=0
        speed.vel_left=0.25
        self.pub.publish(speed)
        rospy.sleep(0.5)
        

    def delta_distance(self) -> float:
        
        if len(self.average_distances) == 5:
            self.average_distances.append(self.distance)
            self.average_distances.pop(0)
        else:
            self.average_distances.append(self.distance)

        return numpy.average(self.average_distances)
    
    def history(self) -> list:
                
                    ############### 8X8 log ################

        if len(self.prev_bits)<8:
            self.prev_bits.append(self.bits)
        else:
            self.prev_bits.pop(0)
            self.prev_bits.append(self.bits)

        return self.prev_bits
    
    def run(self):
        rate = rospy.Rate(20)
        last_time = time.time() - 0.002
        prev_e = 0.0
        prev_int = 0.0

        while not rospy.is_shutdown():
            while self.distance == 0.0 :
                rate.sleep()

        #roboti käivitamisel peab pisut ootama kuni kõik vajalikud sensorid käivitatakse,
        #vastasel juhul saab programm vale andmeid ja hakkab nende põhjal tegema mitte vajalike operatsioone
            
            if self.delta_distance() < 0.25: ########### OB_AVOID ############
                print(f'avoiding object {self.bits} {self.delta_distance()} {self.average_distances}')
                self.ob_avoid()

            if self.bits in Joonebitid:
                delta_t = time.time()- last_time
                v_0, omega, prev_e, prev_int = PID_Controller.PIDController(prev_e, prev_int, delta_t, self.bits)
                speed.vel_right = v_0 + omega
                speed.vel_left = v_0 - omega

                self.pub.publish(speed)
                last_time = time.time()


            else:          ############## MOVE #################
                v_0 = 0.3
                while self.bits not in Joonebitid:
                    if self.bits in Vasakule_90:
                        speed.vel_right = v_0 * 0.75
                        speed.vel_left = -0.25 * (v_0)
                        self.pub.publish(speed)
                    elif self.bits in Paremale_90:
                        speed.vel_right = -0.25 * (v_0)
                        speed.vel_left = v_0 * 0.75
                        self.pub.publish(speed)
                    else:
                        break               

            rate.sleep()       
          

if __name__ == '__main__':
    node = STRONK(node_name='stronk_node')
    node.run()
    rospy.on_shutdown(STRONK.on_shutdown)
    rospy.spin()