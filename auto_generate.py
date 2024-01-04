import sys, threading, time, socket, struct, uuid, os
import json, ast

if not os.path.exists("datalogs-auto"):
    os.mkdir("datalogs-auto")

# This an automated version of the data_receiver_pygame.py file
# It will automatically collect data from the turtlebot and save it to a file given a
# dictionary entry of the form:
#  "label1" : [[action1, deltaT1],[action2, deltaT2],...] 
#  "label2" : [[action1, deltaT1],[action2, deltaT2],...]

# Possitble actions:
# @RNDM: Randomize turtlebot position and orientation
# @ODOM: Record turtlebot odometry
# @0000: Do nothing (Turtlebot stops moving)
# @KILL: Kill the program
# @FRWD: Move turtlebot forward (about 0.22 m/s)
# @LEFT: Turn turtlebot left (about 0.75 rad/s)
# @RGHT: Turn turtlebot right (about -0.75 rad/s)
# @STRT: similar to @ODOM. Retrieves Turtlebot Odometry

# NOTE: The Turtlebot will randomize its position and orientation before each label entry.
    
def parse_line(line):
    # Split the line into key and value parts
    key, value = line.split(':')
    # Remove extra whitespace and the enclosing quotes from the key
    key = key.strip().strip('"')
    # Convert the string representation of the list to an actual list
    value = eval(value.strip())
    return key, value

def read_labels(filename='instructions.txt'):
    data_dict = {}
    with open(filename, 'r') as file:
        for line in file:
            key, value = parse_line(line)
            data_dict[key] = value
    return data_dict

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
        self.debug = False

        # read data labels from file here
        self.prompts = read_labels()
        if self.prompts is None: print("Error encountered reading file. Prompts list is empty. Closing."); sys.exit(0)

        """
        The program will iteratively go through each label and gather data. Before each automated data collection, the program will randomize the turtlebot's position and orientation.
        """

        # set number of iterations here:
        self.total_passes = 2 # default value: 10. Will need hundred of thousands of data points to train a good model. Each pass is slow, so this is a good number to start with.

        # multithreading processes
        self.listener_thread = threading.Thread(target = self.data_listener)
        self.listener_thread.start()

    def data_listener(self):
        
        #  {"label1" : [[action1, deltaT1],[action2, deltaT2],...], "label2" : [[action1, deltaT1],[action2, deltaT2],...], ...}
        data_points = []
        while self.total_passes:

            print('==================================' + '\n' + 'Starting new pass...')
            
            for entry in self.prompts.items():

                print(time.ctime() + ': ' + f'clearing data points... {data_points}')
                data_points = []
                print(time.ctime() + ': ' + f'Data points cleared. {data_points}')
                print(time.ctime() + ': ' + f'Processing: {entry}')
                
                print(time.ctime() + ': ' + 'Randomizing...')
                self.send_data('@RNDM')
                self.receive_data() # wait for ACK
                print(time.ctime() + ': ' + 'Randomized. Starting data collection...')
                
                # receive initial conditions
                print(time.ctime() + ': ' + f'Starting new data collection... {entry}; pass: {self.total_passes}')
                self.send_data('@STRT')
                x = ast.literal_eval(self.receive_data().decode()) # preliminary data point
                data_points.append(x)
                print(time.ctime() + ': ' + f'Initial conditions received. {x}')

                
                for instruction in entry[1]:

                    action = instruction[0]
                    dt = instruction[1]
                    
                    if instruction[0] == '@KILL': print('Killing...'); self.send_data('@0000'); self.send_data('@KILL'); break
                    
                    elif instruction[0] == '@RAND':

                        print(time.ctime() + ': ' + 'Randomizing...')
                        self.send_data('@RAND')
                        self.receive_data() # wait for ACK
                        print(time.ctime() + ': ' + 'Randomized.')
                        time.sleep(0.1)

                    elif instruction[0] == '@ODOM':

                        print(time.ctime() + ': ' + 'Reading Odometer data...')
                        self.send_data('@ODOM')
                        data_points.append(ast.literal_eval(self.receive_data().decode()))
                        print(time.ctime() + ': ' + f'Odometry Data saved. Current buffer: {data_points}')
                    
                    else:

                        print(time.ctime() + ': ' + f'Sending {action} for {dt} seconds...')
                        self.send_data(action)
                        time.sleep(dt)
                        self.send_data('@0000')
                        self.send_data('@ODOM')
                        data_points.append(ast.literal_eval(self.receive_data().decode()))
                        print(time.ctime() + ': ' + f'Data saved. Current buffer: {data_points}')
                        print(time.ctime() + ': ' + 'Done.')

                self.send_data('@0000')
                fname = self.generate_random_filename()
                with open(os.path.join(os.getcwd(), 'datalogs-auto', fname), 'w') as f:
                    print(time.ctime() + ': ' + f'SAVED:', str({'label':entry[0], 'data_points':data_points}))
                    f.write(str({'label':entry[0], 'data_points':data_points}))
                    print(time.ctime() + ': ' + f'Data written to {fname}.')

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
    
x = turtlebot_controller_automatic()