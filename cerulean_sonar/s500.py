from ast import FormattedValue
import rclpy
from rclpy.node import Node
from brping import Ping1D
from sensor_msgs.msg import Range
class s500_sonar:
    def __init__(self, parameters):
        self.get_logger().info("Initializing s500 sonar device...")
        
        # Initialize parameters
        global params
        params = parameters
        
        # Sonar initialization
        global sonar
        sonar = Ping1D()

        if params.comm_type == 'serial':
            self.get_logger().info("Attempting to connect to device at port %s over serial.\n", params.device_port)
            sonar.connect_serial(params.device_port, params.baudrate)
        elif params.comm_type == 'udp':
            self.get_logger().info("Attempting to connect to device at %s:%s over udp.\n", params.udp_address)
            sonar.connect_udp(params.udp_address, params.udp_port)
        else:
            self.get_logger().info("Error when attempting connection. Ensure the comm_type parameter is either serial or udp. \n")
    
    def initialize(self):
        if sonar.initialize() is False:
            return False
        else:
             # Create ROS Publisher if initalization succedeed 
            self.publisher_ = self.create_publisher(Range, params.sonar_topic, 10)
            self.timer = self.create_timer(1.0 / params.frequency, self.sonar_callback)
            return True
        
    def sonar_callback(self):
        data = sonar.get_distance()
        range_msg = Range()
        range_msg.header.frame_id = params.frame
        range_msg.radiation_type = 1
        range_msg.field_of_view = params.fov
        range_msg.min_range = params.min_range
        range_msg.max_range = params.max_range
        range_msg.range = data["distance"] / 1000.0
        confidence = data["confidence"]
        self.publisher_.publish(range_msg)
        self.get_logger().info(f"Distance: {range_msg.range: .4f}\tConfidence: {confidence: .2f}")

        