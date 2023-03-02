import rospy
from duckietown.dtros import DTROS, NodeType

import my_publisher_node
import my_subscriber_node

class ROSPROG(DTROS):
    def __init__(self, node_name):
        super(ROSPROG, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)