import sys, threading, time, socket, struct, uuid, os
import json

if not os.path.exists("datalogs"):
    os.mkdir("datalogs")

# This an automated version of the data_receiver_pygame.py file
# It will automatically collect data from the turtlebot and save it to a file given a
# dictionary entry of the form:
#  "label1" : [[action1, deltaT1],[action2, deltaT2],...] 
#  "label2" : [[action1, deltaT1],[action2, deltaT2],...]

def read_labels(labels_file='data_labels.txt'):

    entries = {}
    
    try:
        with open(labels_file, 'r') as f:
            for line in f.readlines():
                line = line.strip()
                if line == '': continue
                entries.update(json.loads(line)) 
        return entries
    
    except Exception as e:
        print(f'ERROR: {e}')
        return None
    
class turtlebot_controller_automatic:

    def __init__(self):

        # socket programming stuff
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.bind(('0.0.0.0', 50000))
        self.server_sock.listen(0)
        print(f'Server Listening on port 50000')

        self.client, client_address = self.server_sock.accept()
        print('Client copnnected from', client_address)

        # debug locks:
        self.debug = True

        # read data labels from file here
        self.prompts = read_labels()
        if self.prompts is None: print("Error encountered reading file. Prompts list is empty. Closing."); sys.exit(0)

        """
        The program will iteratively go through each label and gather data. Before each automated data collection, the program will randomize the turtlebot's position and orientation.
        """

        # set number of iterations here:
        self.total_passes = 10 # default value: 10. Will need hundred of thousands of data points to train a good model. Each pass is slow, so this is a good number to start with.

        # multithreading processes
        self.listener_thread = threading.Thread(target = self.data_listener)
        self.listener_thread.start()

    def data_listener(self):
        
        while self.total_passes:

            data_points = []
            for entry in self.prompts.items():
                
                # receive initial conditions
                self.send_data('@STRT')
                x = self.receive_data().decode()
                data_points.append(x)

                for instruction in entry[1]:
                    self.send_data(instruction[0])
                    time.sleep(instruction[1])
                    if instruction[0] == '@STOP': break
                    elif instruction[0] == '@STRT': continue
                    elif instruction[0] == '@ODOM': continue
                    elif instruction[0] == '@TURN': continue
                    elif instruction[0] == '@RAND': continue
                    elif instruction[0] == '@RSET': continue
                    elif instruction[0] == '@RPOS': continue
                    elif instruction[0] == '@RVEL': continue
                    elif instruction[0] == '@RANG': continue
                    elif instruction[0] == '@RANG': continue
                    x = self.receive_data().decode()
                    data_points.append(x)
            self.total_passes -= 1
    
    def generate_random_filename(self):
        random_filename = str(uuid.uuid4().hex)[:16]
        return random_filename
    
    def send_data(self, data: bytes):

        if isinstance(data, str):
            data = data.encode()

        # NOTE: data may or may not be in string format.
            
        length_bytes = struct.pack('!I', len(data))
        
        if self.debug: print('[S] Sending byte length...')
        self.client.sendall(length_bytes)
        ack = self.client.recv(2) # wait for other side to process data size
        if ack != b'OK': print(f'[S] ERROR: unmatched send ACK. Received: {ack}')
        if self.debug: print('[S] ACK good')

        if self.debug: print('[S] Sending data...')
        self.client.sendall(data) # send data
        if self.debug: print('[S] Data sent; waiting for ACK...')
        ack = self.client.recv(2) # wait for other side to process data size
        if ack != b'OK': print(f'[S] ERROR: unmatched send ACK. Received: {ack}')
        if self.debug: print('[S] ACK good. Data send success.')

    def receive_data(self):

        # NOTE: Returns data in BINARY. You must decode it on your own

        if self.debug: print('[R] Waiting for byte length...')
        length_bytes = self.client.recv(4)
        length = struct.unpack('!I', length_bytes)[0]
        if self.debug: print(f'[R] Byte length received. Expecting: {length}')
        data, data_size = b'', 0

        self.client.send(b'OK') # allow other side to send over the data
        if self.debug: print(f'[R] ACK sent.')
        while data_size < length:

            chunk_size = min(2048, length - data_size)
            data_size += chunk_size
            data += self.client.recv(chunk_size)
            if self.debug: print(f'[R] RECV {chunk_size}')

        if self.debug: print('[R] Transmission received successfull. Sending ACK')       
        self.client.send(b'OK') # unblock other end
        if self.debug: print('[R] ACK sent.')
        return data # up to user to interpret the data