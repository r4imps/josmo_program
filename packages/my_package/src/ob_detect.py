import time
import rospy
from duckietown_msgs.msg import WheelsCmdStamped
speed = WheelsCmdStamped()
pub = rospy.Publisher('/josmo/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)

