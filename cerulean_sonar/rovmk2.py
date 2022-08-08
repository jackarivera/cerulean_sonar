from asyncio.log import logger
import rclpy
from rclpy.node import Node
from brping import Ping1D
from sensor_msgs.msg import Range
class rovmk2_sonar:
    def __init__(self, params, sonar_node, autosyncEnabled):
        # Handle master node
        global node   
        node = sonar_node

        # Setup logger
        global logger
        logger = sonar_node.get_logger()
    