from cerulean_sonar.s500 import s500_sonar
from cerulean_sonar.rovmk2 import rovmk2_sonar
from cerulean_sonar.rovmk3 import rovmk3_sonar
import rclpy
from rclpy.node import Node
from brping import Ping1D
from sensor_msgs.msg import Range


class sonar_node(Node):
    def __init__(self):
        super().__init__("sonar_node")
        self.get_logger().info("Starting S500 Sonar Node...")
    
        # Create dictionary of node parameters to easily pass into sonar object
        params = {
            "device_port": self.declare_parameter('device_port', '/dev/ttyACM0'),
            "device_type": self.declare_parameter('device_type', 's500'), # rovmk2, rovmk2a (autosync), rovmk3, s500
            "baudrate": self.declare_parameter('baudrate', 115200),
            "frequency": self.declare_parameter('frequency', 10), # Amount of times per second its published
            "sonar_topic": self.declare_parameter('sonar_topic', 'sonar'),
            "frame": self.declare_parameter('frame', 'sonar'),
            "fov": self.declare_parameter('fov', 0.3),
            "min_range": self.declare_parameter('min_range', 0.5),
            "max_range": self.declare_parameter('max_range', 50.0),
            "comm_type": self.declare_parameter('comm_type', 'serial'), # serial or udp
            "udp_address": self.declare_paremeter('udp_address', '0.0.0.0'),
            "udp_port": self.declare_parameter('udp_port', 12345)
        }

        if params.device_type == 's500':
            sonar = s500_sonar(params)
        elif params.device_type == 'rovmk2':
            sonar = rovmk2_sonar(params, False)
        elif params.device_type == 'rovmk2a':
            sonar = rovmk2_sonar(params, True)
        elif params.device_type == 'rovmk3':
            sonar = rovmk3_sonar(params)
        else:
            print('Error when initializing sonar device type. Check config file for misspelled or non supported device_type')

       

def main(args=None):
    rclpy.init(args=args)
    node = sonar_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()