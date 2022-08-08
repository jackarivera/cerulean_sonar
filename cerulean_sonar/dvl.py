import rclpy
from rclpy.node import Node
from brping import Ping1D
from sensor_msgs.msg import Range
class dvl:
    def __init__(self, params, sonar_node):
        # Handle master node
        global node   
        node = sonar_node

        # Setup logger
        global logger
        logger = sonar_node.get_logger()
    