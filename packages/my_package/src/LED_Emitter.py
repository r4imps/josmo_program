#!/usr/bin/env python3
import time

import rospy


from std_msgs.msg import String, ColorRGBA
from duckietown_msgs.msg import LEDPattern
from duckietown.dtros import DTROS, TopicType, NodeType

LEDS= LEDPattern()
SET_RGBA= ColorRGBA()
global colors

class LEDDriverNode(DTROS):
    """Node for controlling LEDs.
    Calls the low-level functions of class :obj:`RGB_LED` that creates the PWM
    signal used to change the color of the LEDs. The desired behavior is specified by
    the LED index (Duckiebots and watchtowers have multiple of these) and a pattern.
    A pattern is a combination of colors and blinking frequency.
    Duckiebots have 5 LEDs that are indexed and positioned as following:
    +------------------+------------------------------------------+
    | Index            | Position (rel. to direction of movement) |
    +==================+==========================================+
    | 0                | Front left                               |
    +------------------+------------------------------------------+
    | 1                | Rear left                                |
    +------------------+------------------------------------------+
    | 2                | Top / Front middle                       |
    +------------------+------------------------------------------+
    | 3                | Rear right                               |
    +------------------+------------------------------------------+
    | 4                | Front right                              |
    +------------------+------------------------------------------+
    """

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(LEDDriverNode, self).__init__(node_name=node_name, node_type=NodeType.DRIVER)

        self.pub = rospy.Publisher("/josmo/led_emitter_node/led_pattern", LEDPattern, queue_size=10)


    def run(self):
        
        if not rospy.is_shutdown():
            SET_RGBA.r=1.0
            SET_RGBA.g=1.0
            SET_RGBA.b=1.0
            SET_RGBA.a=1.0

            #colors[0]=SET_RGBA
            LEDS.rgb_vals.append(SET_RGBA)
            self.pub.publish(LEDS)


    
       

if __name__ == "__main__":
    # Create the LEDdriverNode object
    led_driver_node = LEDDriverNode(node_name="led_driver_node")
    # Keep it spinning to keep the node alive
    led_driver_node.run()
    rospy.spin()