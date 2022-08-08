import serial

class comm_serial:
    def __init__(self, parameters):
        # Setup parameters
        global params
        params = parameters

        # Create the global connection object
        global comm
        comm = serial.Serial()

        # Configure serial parameters
        comm.baudrate = params.get("baudrate")
        comm.port = params.get("device_port")

    def openComm(self):
        comm.open()

    def closeComm(self):
        comm.close()

    def readByte(self):
        place = "holder"

    def isOpen(self):
        return comm.is_open