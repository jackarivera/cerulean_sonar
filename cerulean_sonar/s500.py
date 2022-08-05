import rclpy
from rclpy.node import Node
from brping import Ping1D
class s500_node(Node):
    def __init__(self):
        super().__init__("s500")
        self.get_logger().info("Starting S500 Sonar Node...")
        
        # Node Parameters
        device_port = self.declare_parameter('device_port', '/dev/ttyACM0')
        baudrate = self.declare_parameter('baudrate', 115200)
        frequency = self.declare_parameter('frequency', 10)
        frame = self.declare_parameter('frame', 'sonar')
        fov = self.declare_parameter('fov', 0.3)
        min_range = self.declare_parameter('min_range', 0.5)
        max_range = self.declare_parameter('max_range', 50.0)
        comm_type = self.declare_parameter('comm_type', 'serial') # serial or udp

        # Sonar initialization
        sonar = Ping1D()

        if comm_type == 'serial':
            print("Attempting to connect to device at port %s.\n", device_port)
            sonar.connect_serial(device_port, baudrate)

            if sonar.initialize() is False:
                print("Failed to connect to device %s over serial\n", device_port)
                print("Shutting down this node...\n")
                self.destroy_node()
                rclpy.shutdown()
            
            print("Successfully connected to device at %s", device_port)
        elif comm_type == 'udp':
            print("UDP Connections not yet implemented. \n") 
        else:
            print("Unknown error when attempting connection. Ensure the comm_type parameter is either SERIAL or UDP. \n")     

def main(args=None):
    rclpy.init(args=args)
    node = s500_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()