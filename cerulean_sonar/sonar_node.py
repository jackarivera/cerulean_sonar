from cerulean_sonar.s500 import s500_sonar
from cerulean_sonar.rovmk2 import rovmk2_sonar
from cerulean_sonar.rovmk3 import rovmk3_sonar
import rclpy
from rclpy.node import Node


class sonar_node(Node):
    def __init__(self):
        super().__init__("sonar_node")
        self.get_logger().info("Starting Sonar Node...")
    
        # Create dictionary of node parameters to easily pass into sonar object
        parameters = {
            "device_port": self.declare_parameter('device_port', '/dev/ttyACM0').value,
            "device_type": self.declare_parameter('device_type', 's500').value, # rovmk2, rovmk3, s500
            "baudrate": self.declare_parameter('baudrate', 115200).value,
            "frequency": self.declare_parameter('frequency', 10).value, # Amount of times per second its published
            "sonar_topic": self.declare_parameter('sonar_topic', 'sonar').value,
            "frame": self.declare_parameter('frame', 'sonar').value,
            "fov": self.declare_parameter('fov', 0.3).value,
            "min_range": self.declare_parameter('min_range', 0.5).value,
            "max_range": self.declare_parameter('max_range', 50.0).value,
            "comm_type": self.declare_parameter('comm_type', 'serial').value, # serial or udp
            "udp_address": self.declare_parameter('udp_address', '0.0.0.0').value,
            "udp_port": self.declare_parameter('udp_port', 12345).value
        }


        # Create device type and attempt connection
        if parameters.get("device_type") == 's500':
            sonar = s500_sonar(parameters, self)
            
            if sonar.initialize() is False:
                if parameters.get("comm_type") == "serial":
                    self.get_logger().info("Failed to connect to device %s over serial.\n" % parameters.get("device_port"))
                elif parameters.get("comm_type") == "udp":
                    self.get_logger().info("Failed to connect to device %s:%s over udp.\n" % (parameters.get("udp_address"), parameters.get("udp_port")))
                
                self.get_logger().info("Shutting down this node...\n")
                self.destroy_node()
                rclpy.shutdown()
            else:
                self.get_logger().info("Successfully connected to device at %s" % parameters.get("device_port"))
        elif parameters.get("device_type") == 'rovmk2':
            sonar = rovmk2_sonar(parameters, self, False)

            if parameters.get("comm_type") == "serial":
                if sonar.checkSerialComm() is False:
                    self.get_logger().info("Shutting down this node...\n")
                    self.destroy_node()
                    rclpy.shutdown()
            elif parameters.get("comm_type") == "udp":
                if sonar.checkUdpComm() is False:
                    self.get_logger().info("Shutting down this node...\n")
                    self.destroy_node()
                    rclpy.shutdown()
        elif parameters.get("device_type") == 'rovmk3':
            sonar = rovmk3_sonar(parameters, self)
        else:
            self.get_logger().info('Error when initializing sonar device type. Check config file for misspelled or non supported device_type')

       

def main(args=None):
    rclpy.init(args=args)
    node = sonar_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()