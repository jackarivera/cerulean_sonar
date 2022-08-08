from cerulean_sonar.comm_serial import comm_serial
from cerulean_sonar.comm_udp import comm_udp
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
            logger.info("Opening serial connection at port %s\n" % params.get("device_port"))
            comm = comm_serial(params)
            comm.openComm()
        elif params.get("comm_type") == "udp":
            logger.info("Opening UDP connection at port %s:%s\n" % (params.get("udp_address"), params.get("udp_port")))
            comm = comm_udp(params)
        else:
            logger.info("Error opening a connection with the device. Check that your comm_type is correct.\n")

    def checkSerialComm(self):
        if not comm.isOpen():
            logger.info("Error. Serial communication is not open or could not be opened. Check device port and baudrate.\n")
            return False
        else:
            logger.info("Successfully connected to serial port %s.\n" % params.get("device_port"))
            self.publisher_ = node.create_publisher(Range, params.get("sonar_topic"), 10)
            self.timer = node.create_timer(1.0 / params.get("frequency"), self.sonar_callback)
            return True
            
    def checkUdpComm(self):
        if not comm.isOpen():
            logger.info("Error. UDP communication could not be established.\n")
            return False
        else:
            logger.info("Successfully established UDP connection at %s:%s.\n" % (params.get("udp_address"), params.get("udp_port")))
            self.publisher_ = node.create_publisher(Range, params.get("sonar_topic"), 10)
            self.timer = node.create_timer(1.0 / params.get("frequency"), self.sonar_callback)
            return True

    def sonar_callback(self):
        data = comm.readByte()
        logger.info(data)