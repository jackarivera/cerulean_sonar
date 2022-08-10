from brping import pinghf
from sensor_msgs.msg import Range
class s500_sonar:
    def __init__(self, parameters, sonar_node):
        # Handle the passed in node
        global node
        node = sonar_node

        # Configure Logger
        global logger
        logger = sonar_node.get_logger()
        logger.info("Initializing s500 sonar device...")
        
        # Initialize parameters
        global params
        params = parameters
        
        # Sonar initialization
        global sonar
        sonar = pinghf.Pinghf()
        
        if params.get("comm_type") == 'serial':
            logger.info("Attempting to connect to device at port %s over serial.\n" % params.get("device_port"))
            sonar.connect_serial(params.get("device_port"), params.get("baudrate"))
        elif params.get("comm_type") == 'udp':
            logger.info("Attempting to connect to device at %s:%s over udp.\n" % (params.get("udp_address"), params.get("udp_port")))
            sonar.connect_udp(params.get("udp_address"), params.get("udp_port"))
        else:
            logger.info("Error when attempting connection. Ensure the comm_type parameter is either serial or udp. \n")
    
    def initialize(self):
        if sonar.initialize() is False:
            return False
        else:
             # Create ROS Publisher if initalization succedeed 
            self.publisher_ = node.create_publisher(Range, params.get("sonar_topic"), 10)
            self.timer = node.create_timer(1.0 / params.get("frequency"), self.sonar_callback)
            return True
        
    def sonar_callback(self):
        distance = -1.0 # Set to -1 for people to be able to handle if we aren't able to get a range distance

        try:
            distance = sonar.get_profile()["distance"]# Try and get distance from sonar profile
        except:
            logger.info("Unable to get sonar measurement before timeout. Try increasing timeout in ping1d file. Returning -1.0 for distance")
        #sos = sonar.get_distance2()
        range_msg = Range()
        range_msg.header.frame_id = params.get("frame")
        range_msg.radiation_type = 1
        range_msg.field_of_view = params.get("fov")
        range_msg.min_range = params.get("min_range")
        range_msg.max_range = params.get("max_range")
        range_msg.range = distance / 1.0 # returns distance in mm
        #confidence = data["confidence"]
        self.publisher_.publish(range_msg)
        print(distance)
        #print(sos)
        