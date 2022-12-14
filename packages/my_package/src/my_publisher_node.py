#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from duckietown_msgs.msg import WheelsCmdStamped
from smbus2 import SMBus
speed = WheelsCmdStamped()
class MyPublisherNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        self.pub = rospy.Publisher('/josmo/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)

#    def on_shutdown(self):
#        speed.vel_left = 0
#        speed.vel_right = 0
#        self.pub.publish(speed)
#        rospy.on_shutdown()

    def run(self):
        flag = 0
        # publish message every 1 second
        rate = rospy.Rate(20) # 1Hz
        while not rospy.is_shutdown():

            read = SMBus(1).read_byte_data(0x3e, 0x11)
            #read= bin(read)

            # oooXXooo
            if read == 231:
                speed.vel_left = 0.4
                speed.vel_right = 0.4
                print("OTSE")
                
            # ooooXooo
            if read == 239:
                speed.vel_left = 0.3
                speed.vel_right = 0.38
                print("NATUKE VASAKULE")
            # ooooXXoo    
            if read == 207 :
                speed.vel_left = 0.1
                speed.vel_right = 0.2
                print("VASAKULE")

            # oooooXXo    
            if read == 159 :
                speed.vel_left = 0
                speed.vel_right = 0.15
                print("R2IGELT VASAKULE")

            if read == 127 :
                speed.vel_left = -0.20
                speed.vel_right = 0.225
                print("R2IGEMALT VASAKULE")
                rospy.sleep(0.15)
################################################
            # oooXoooo                   
            if read == 247 :
                speed.vel_left = 0.3
                speed.vel_right = 0.38
                print("NATUKE PAREMALE")
            #ooXXoooo
            if read == 243 :
                speed.vel_left = 0.2
                speed.vel_right = 0.1
                print("PAREMALE")

            if read == 249 :
                speed.vel_left = 0.15
                speed.vel_right = 0
                print("R2IGELT PAREMALE")  

            if read == 254 :
                speed.vel_left = 0.225
                speed.vel_right = -0.20
                print("R2IGEMALT PAREMALE")
                rospy.sleep(0.15)



        
            rospy.sleep(0.02)





            # oooooooo
            if read == 255 :
                speed.vel_left = 0
                speed.vel_right = 0
                print("STOP")

                
            print(read)
            
            self.pub.publish(speed)
            rate.sleep()

if __name__ == '__main__':
    # create the node
    node = MyPublisherNode(node_name='my_publisher_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()