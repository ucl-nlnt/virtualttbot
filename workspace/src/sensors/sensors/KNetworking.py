import socket
import threading
import time
import sys
from collections import deque
import struct
import random
from simple_chalk import chalk

###
#   NOTE on USAGE: DataBridgeClient assumes that the Server is already running. Please ensure that it's setting up first!
###

def find_optimal_data_bisection(packet_size: int):

    tcp_max = 64000 # ensure that the limit is not reached
    return tcp_max - (tcp_max % packet_size)

def create_chance_for_error(error_type = socket.timeout, chance = 0.5): # used to simulater errors

    if random.random() <= chance: raise error_type

def send_data_subroutine(socket_object: socket.socket, data: bytes, 
                         destination_port: int, debug: bool = False, send_acks: bool = False,
                         packet_size: int = 1460, sim_errors: bool = False):

    if isinstance(data, str):
        data = data.encode()

    if not isinstance(data, bytes):
        raise TypeError("object.send_data() requires String or Bytes as input.")

    try:

        length_bytes = struct.pack('!I', len(data))
        if debug: print(f'[S: Destination:{destination_port}] Sending byte length...')

        # Bisect into segments if length is greater than ~64KB.
        data_length = len(data)
        remainder = data_length % find_optimal_data_bisection(packet_size) # may not by 64KB; depends on packet size

        socket_object.sendall(length_bytes) # tell destination total file size
        for i in range(0,data_length - remainder,packet_size):
            if send_acks:
                ack = socket_object.recv(2) # wait for other side to process data size
                if ack != b'OK': print(f'[S: Destination:{destination_port}] ERROR: unmatched send ACK. Received: {ack}')
                if debug: print(f'[S: Destination:{destination_port}] ACK good')

            if debug: print(f'[S: Destination:{destination_port}] Sending data...')
            socket_object.sendall(data[i:i+packet_size]) # send data
            if send_acks:
                ack = socket_object.recv(2) # wait for other side to process data size
                if ack != b'OK': print(f'[S: Destination:{destination_port}] ERROR: unmatched send ACK. Received: {ack}')
                if debug: print(f'[S: Destination:{destination_port}] ACK good')

        if remainder: 
            socket_object.sendall(data[data_length - remainder:])
            if debug: print(f"sending Remainder {remainder}")
            if send_acks:
                ack = socket_object.recv(2) # wait for other side to process data size
                if ack != b'OK': print(f'[S: Destination:{destination_port}] ERROR: unmatched send ACK. Received: {ack}')
                if debug: print(f'[S: Destination:{destination_port}] ACK good')

        if debug: print(f"Transmission done.")
        return "@SNOK"
    
    except socket.timeout as errormsg:
        print(f"[Sending Subroutine -> Destination: {destination_port}]: Timeout occured.")
        print(errormsg)
        return "@SNTO"
    
    except struct.error as errormsg:
        print(f"[Sending Subrouting -> Destination: {destination_port}]: Struct error occured.")
        print(errormsg)
        return "@SNSE"
    
    except BrokenPipeError as errormsg:
        print(f"[Sending Subroutine -> Destination: {destination_port}]: BrokenPipeError occured.")
        print(errormsg)
        return "@SNBP"
    
    except ConnectionResetError as errormsg:
        print(f"[Sending Subroutine -> Destination: {destination_port}]: ConnectionResetError occured.")
        print(errormsg)
        return "@SNCR"
        
    except ConnectionError as errormsg:
        print(f"[Sending Subroutine -> Destination: {destination_port}]: ConnectionError occured.")
        print(errormsg)
        return "@SNCE"
    
    

    
def receive_data_subroutine(socket_object: socket.socket, destination_port: int,
                            debug: bool = False, send_acks: bool = False, 
                            packet_size: int = 1460, sim_errors: bool = False):

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

            chunk_size = min(packet_size, length - data_size)
            chunk_data = b''

            # Section added by GPT-4; apparently, socket.socket.recv() sometimes received partial datagrams only. weird.
            # the following while loop made sure that each iteration or each assumed TCP packet gave the complete data

            while len(chunk_data) < chunk_size:
                packet = socket_object.recv(chunk_size - len(chunk_data))
                if not packet:
                    raise RuntimeError("Connection closed by the server")
                chunk_data += packet

            data += chunk_data
            data_size += len(chunk_data)
            if debug: print(f"[R: Destination:{destination_port}] Received chunk of size {len(chunk_data)}. Total received: {data_size}")
            
        if send_acks:
            if debug: print(f'[R: Destination:{destination_port}] Transmission received successfull. Sending ACK')       
            socket_object.send(b'OK') # unblock other end
            if debug: print(f'[R: Destination:{destination_port}] ACK sent.')

        return data # up to user to interpret the data
    
    except struct.error as errormsg:
        print(f"[Receiving Subrouting -> Destination: {destination_port}]: Struct error occured.")
        print(errormsg)
        return "@RCSE"

    except socket.timeout as errormsg:
        print(f"[Receiving Subroutine -> Destination: {destination_port}]: Timeout occured.")
        print(errormsg)
        return "@RCTO"
    
    except ConnectionResetError as errormsg:
        print(f"[Receiving Subroutine -> Destination: {destination_port}]: ConnectionResetError occured.")
        print(errormsg)
        return "@RCCR"

    except BrokenPipeError as errormsg:
        print(f"[Receiving Subroutine -> Destination: {destination_port}]: BrokenPipeError occured.")
        print(errormsg)
        return "@RCBP"
    
    except ConnectionError as errormsg:
        print(f"[Receiving Subroutine -> Destination: {destination_port}]: ConnectionError occured.")
        print(errormsg)
        return "@RCCE"
    

class DataBridgeClient_TCP:
    
    # TODO: Add timeout functions and heartbeat

    def __init__(self, destination_port:int, destination_ip_address:str = "127.0.0.1", enable_acks:bool = False,debug:bool = False, packet_size:int = 1460):

        self.destination_port = destination_port
        self.destination_ip_address = destination_ip_address
        self.debug = debug
        self.enable_acks = enable_acks
        self.packet_size = packet_size
        self.client = None

        # SET TO FALSE TO TURN OFF RANDOM ERRORS
        self.simulate_errors = False

        self.reconnect_to_server()

    def reconnect_to_server(self):

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

    def disconnect_from_server(self):
        
        self.client.close()

    def send_data(self, data: bytes):

        code = send_data_subroutine(socket_object=self.client, data=data, destination_port=self.destination_port, debug=self.debug, send_acks=self.enable_acks, packet_size=self.packet_size)
        if code == '@SNOK': return # no error; continue
    
        elif code == '@SNBP' or code == '@SNSE': # BrokenPipe or Struct.error

            # Other end closed the connection. Destroy the current connection object
            try:
                self.client.close()
            except:
                pass

        # TODO: find a way to restart data transfer on that frame
            
        elif code == '@SNCE' or code == '@SNCR' or code == "@SNTO": # ConnectionError, ConnectionResetError, TimeOut
            
            # restart the connection maybe?
            try:
                self.client.close()
            except:
                pass

            self.reconnect_to_server()
            self.send_data(data=data) # re-attempt to send data

    def receive_data(self):

        return receive_data_subroutine(self.client, destination_port=self.destination_ip_address, debug=self.debug, send_acks=self.enable_acks, packet_size=self.packet_size)

class DataBridgeServer_TCP:

    def __init__(self, port_number: int = 50000, enable_acks = False, debug:bool = False, packet_size: int = 1460):
        
        self.port_number = port_number
        
        self.enable_acks = enable_acks
        self.debug = debug
        self.packet_size = packet_size
        # TURN OFF TO DISABLE RANDOM ERRORS
        self.simulate_errors = False
        self.client = None
        self.client_address = None
        
        self.open_server_port()

    def open_server_port(self):
        
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # WARNING: MIGHT NEED TO CHECK IF PORT IS OCCUPIED
        self.server_sock.bind(('0.0.0.0', self.port_number))
        self.server_sock.listen(0)
        
        print(f"[DataBridgeServer TCP : {self.port_number}] Server started.")
        print('Waiting for Turtlebot Connection!')
        
        self.client, self.client_address = self.server_sock.accept()
        print(f"[DataBridgeServer TCP : {self.port_number}] Client connected from {self.client_address}.")
        print(f"[DataBridgeServer TCP : {self.port_number}] Setup complete.")

    def close_server_port(self):

        self.client.shutdown(socket.SHUT_RDWR)
        self.client.close()

    def cleanup(self):
        self.close_server_port()

    def send_data(self, data: bytes):

        code = send_data_subroutine(socket_object=self.client, data=data, destination_port=self.port_number, debug=self.debug, send_acks=self.enable_acks, packet_size=self.packet_size)
        if code == '@SNOK': return # no error; continue
    
        elif code == '@SNBP' or code == '@SNSE': # BrokenPipe or Struct.error

            # Other end closed the connection. Destroy the current connection object
            self.cleanup()

        # TODO: find a way to restart data transfer on that frame
            
        elif code == '@SNCE' or code == '@SNCR' or code == "@SNTO": # ConnectionError, ConnectionResetError, TimeOut
            
            # restart the connection maybe?
            self.close_server_port()        # close current instance
            self.open_server_port()         # restart server port instance
            self.send_data(data = data)     # restart data transfer

    def receive_data(self):

        data = receive_data_subroutine(socket_object=self.client, destination_port=self.port_number, debug=self.debug, send_acks=self.enable_acks, packet_size=self.packet_size)
        if isinstance(data, bytes):
            return data
        
        if data == '@RCBP' or data == '@RCSE': # BrokenPipe or Struct.error
            
            # Other end closed the connection. Destroy the current connection object
            self.cleanup()

        elif data == '@RCCE' or data == '@RCTO' or data == '@RCCR': # ConnectionError
            
            self.close_server_port()        # close current instance
            self.open_server_port()         # restart server port instance
            self.send_data(data = data)     # restart data transfer