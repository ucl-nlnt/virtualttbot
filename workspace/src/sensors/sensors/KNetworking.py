import socket

class DataBridgeClient_TCP:

    def __init__(self, to_port : int = 50000, to_ip : str = "127.0.0.1"):
        self.client = socket.socket()
        self.destination_ip = to_ip
