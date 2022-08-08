from cerulean_sonar.comm_serial import comm_serial
import rclpy
from rclpy.node import Node
from brping import Ping1D
from sensor_msgs.msg import Range
import serial

class rovmk2_sonar:
    def __init__(self, parameters, sonar_node, autosyncEnabled):
        # Handle master node
        global node   
        node = sonar_node

        # Setup logger
        global logger
        logger = sonar_node.get_logger()

        # Setup parameters
        global params
        params = parameters

        # Create the global connection object
        global comm

        if params.get("comm_type") == "serial":
            logger.info("Opening serial connection at port %s" % params.get("device_port"))
            comm = comm_serial(params)
            comm.openComm()
            
            if not comm.isOpen():
                logger.info("Error. Serial communication is not open or could not be opened. Check device port and baudrate.\n")
            else:
                logger.info("Successfully connected to serial port ")
