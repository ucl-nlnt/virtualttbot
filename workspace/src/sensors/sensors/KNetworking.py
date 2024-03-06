import socket
import threading
import time
import sys
from collections import deque
import struct

class DataBridgeClient_UDP: # CURRENTLY NOT WORKING, DO NOT USE

    def __init__(self, to_port : int = 50000, to_ip : str = "127.0.0.1"):
        
        self.client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.destination_ip = to_ip
        self.destination_port = to_port

        self.timeout = False
        self.timeout_generator_is_active = False
        self.timeout_timeframe_ms = 500
        self.max_time = None
        self.timeout_interrupt_thread = threading.Thread(target=self.timeout_generator)
        self.timeout_interrupt_thread.start()

        print(f"[UDP] Client -> {self.destination_ip}:{self.destination_port}")
    
    def timeout_generator(self): # Generated with GPT-4
        
        # MAX RESOLUTION: 100ms

        while True:

            if not self.timeout_generator_is_active:
                time.sleep(0.1)
                continue  # Skip the rest of the loop if not active
            
            if (time.time() - self.last_received_time) * 1000 > self.timeout_timeframe_ms:
                self.timeout = True
                print("[UDP] Timeout occurred.")
                self.timeout_generator_is_active = False # Exit the loop if a timeout occurs, or adapt as needed for your application
            
            time.sleep(0.1)  # Sleep briefly to prevent high CPU usage
        
    def activate_timeout_generator(self, timeout_ms : int = 500):
        
        self.last_received_time = time.time()
        self.timeout_timeframe_ms = timeout_ms
        self.timeout_generator_is_active = True
    
    def deactivate_timeout_generator(self):

        self.last_received_time = None
        self.timeout_timeframe_ms = None
        self.timeout_generator_is_active = False


    def send_to_server(self, data : bytes, chunk_size : int = 1400, timeout_ms : int = 500):

        pass
        excess = len(data) % chunk_size
        
        if excess != 0: data_last = data[len(data)-excess:]
        data = data[:len(data) - excess]
        
        chunks = [data[i:i+chunk_size] for i in range(0, len(data), chunk_size)]
        if excess != 0: chunks += data_last

        chunk_send_successful = False
        for chunk in chunk:
            self.activate_timeout_generator()
            


"""
self.client = socket.socket()
server_ip = server_ip_address

server_port = 50000
flag_connected = False
self.send_lock = True # signals that new data is allowed to be transmitted

try:
    self.client.connect((server_ip, server_port))
    print("[SensorsSubscriber] Connected to server.")
    flag_connected = True

except Exception as e:
    print(e)
    print("[SensorsSubscriber] Error has occured when trying to connect to server.")

if not flag_connected:
    print("[SensorsSubscriber] Exiting program...")
    sys.exit() # Close program if unable to connect to server
"""


###
#   NOTE on USAGE: DataBridgeClient assumes that the Server is already running. Please ensure that it's setting up first!
###


def send_data_subroutine(socket_object: socket.socket, data, destination_port: int, debug: bool = False, send_acks: bool = False):

    if isinstance(data, str):
        data = data.encode()

    if not (isinstance(data, bytes) or isinstance(data, str)):
        raise TypeError("object.send_data() requires String or Bytes as input.")

    # NOTE: data may or may not be in string format.
    try:
        length_bytes = struct.pack('!I', len(data))
        
        if debug: print(f'[S: Destination:{destination_port}] Sending byte length...')
        socket_object.sendall(length_bytes)
        if send_acks:
            ack = socket_object.recv(2) # wait for other side to process data size
            if ack != b'OK': print(f'[S: Destination:{destination_port}] ERROR: unmatched send ACK. Received: {ack}')
            if debug: print(f'[S: Destination:{destination_port}] ACK good')

        if debug: print(f'[S: Destination:{destination_port}] Sending data...')
        socket_object.sendall(data) # send data
        if send_acks:
            ack = socket_object.recv(2) # wait for other side to process data size
            if ack != b'OK': print(f'[S: Destination:{destination_port}] ERROR: unmatched send ACK. Received: {ack}')
            if debug: print(f'[S: Destination:{destination_port}] ACK good')

    except Exception as e:
        print("Send data subroutine failed, port:", destination_port)
        print(e)
        return True
    
def receive_data_subroutine(socket_object: socket.socket, destination_port: int, debug: bool = False, send_acks: bool = False):

        try:
        
            if debug: print(f'[R: Destination:{destination_port}] Waiting for byte length...')
            length_bytes = socket_object.recv(4)
            length = struct.unpack('!I', length_bytes)[0]
            if debug: print(f'[R: Destination:{destination_port}] Byte length received. Expecting: {length}')
            data, data_size = b'', 0

            if send_acks:
                socket_object.send(b'OK') # allow other side to send over the data
                if debug: print(f'[R: Destination:{destination_port}] ACK sent.')

            while data_size < length:

                chunk_size = min(1400, length - data_size)
                data_size += chunk_size
                data += socket_object.recv(chunk_size)
                if debug: print(f'[R: Destination:{destination_port}] RECV {chunk_size}')

            if send_acks:
                if debug: print(f'[R: Destination:{destination_port}] Transmission received successfull. Sending ACK')       
                socket_object.send(b'OK') # unblock other end
                if debug: print(f'[R: Destination:{destination_port}] ACK sent.')

            return data # up to user to interpret the data
        
        except Exception as e:

            print(e)
            return True



class DataBridgeClient_TCP:
    
    # TODO: Add reconnection options; optimized data flow and possibly remove 'OK' ack messages. Redundant in TCP.

    def __init__(self, destination_port:int, destination_ip_address:str = "127.0.0.1", enable_acks:bool = False,debug:bool = False):

        self.destination_port = destination_port
        self.destination_ip_address = destination_ip_address
        self.debug = debug
        self.enable_acks = enable_acks

        self.client = socket.socket()
            
        flag_connected = False

        # Attempt to connect to the server

        try:
            self.client.connect((self.destination_ip_address,self.destination_port))
            print(f'[DataBridgeClient TCP] Connected to {self.destination_ip_address}:{self.destination_port}.')
            flag_connected = True
        except Exception as e:
            print(f'[DataBridgeClient TCP -> {self.destination_ip_address}:{self.destination_port}] encountered an error:')
            print(e)
            flag_connected = False

        print(f'[DataBridgeClient TCP -> {self.destination_ip_address}:{self.destination_port}] : Connection = {flag_connected}')
        
        if not flag_connected: raise ConnectionError("Cannot connect to server.")

    def send_data(self, data:bytes):

        send_data_subroutine(self.client, data=data, destination_port=self.destination_ip_address, debug=self.debug, send_acks=self.enable_acks)

    def receive_data(self):

        return receive_data_subroutine(self.client, destination_port=self.destination_ip_address, debug=self.debug, send_acks=self.enable_acks)

class DataBridgeServer_TCP:

    def __init__(self, port_number: int = 50000, reuse_address: bool = True, enable_acks = False, debug:bool = False):
        
        self.port_number = port_number
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.enable_acks = enable_acks
        self.debug = debug

        if reuse_address: self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.bind(('0.0.0.0', port_number))
        self.server_sock.listen(0)
        
        print(f"[DataBridgeServer TCP : {self.port_number}] Server started.")
        
        self.client, self.client_address = self.server_sock.accept()
        print(f"[DataBridgeServer TCP : {self.port_number}] Client connected from {self.client_address}.")
        print(f"[DataBridgeServer TCP : {self.port_number}] Setup complete.")
    
    def send_data(self, data: bytes):

        send_data_subroutine(socket_object=self.client, data=data, destination_port=self.port_number, debug=self.debug, send_acks=self.enable_acks)

    def receive_data(self):

        return receive_data_subroutine(socket_object=self.client, destination_port=self.port_number, debug=self.debug, send_acks=self.enable_acks)