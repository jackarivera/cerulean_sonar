class comm_udp:
    def __init__(self, parameters):
        # Setup parameters
        global params
        params = parameters

        # Create the global connection object
        global comm