import socket

class comm_udp:
    def __init__(self, parameters):
        # Setup parameters
        global params
        params = parameters

        # Create the global connection object
        global comm

        comm = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        comm.bind((params.get("udp_address"), params.get("udp_port")))  
    
    def readByte(self):
        data = comm.recvfrom(1024)
        return data

    def isOpen(self):
        return comm.is_open