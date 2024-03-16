import pygame
import sys
import threading
import time
import socket
import struct
import uuid
import os
import ast
import json
import base64
import numpy as np
import cv2
import argparse


from KNetworking import DataBridgeServer_TCP

if not os.path.exists("datalogs"):
    os.mkdir("datalogs")

parser = argparse.ArgumentParser(description="Turtlebot3 NLNT terminal-based controller.")

parser.add_argument("--display",type=bool, default=False, help='Enable or disable OpenCV camera window for debugging purposes.')
parser.add_argument("--enable_autorandomizer_from_csv", type=bool, default=False, help="Creates a level 1 or 2 prompt based on a provided CSV file.")
parser.add_argument("--csv_path",type=str, default="NLNT_level1.csv", help="Specifies path to NLNT level 1 natural language label dataset.")
parser.add_argument("--rotate_r_by",type=int, default=0, help="Rotate NLNT image by some amount before saving. Measured in Clockwise rotations.")
parser.add_argument("--disable_log_compression", type=bool, default=False, help="Set to True to save data as raw. Turning this feature off is NOT recommended.")
args = parser.parse_args()

class turtlebot_controller:

    def __init__(self, manual_control=True):

        self.data_buffer = None
        self.current_user = input("enter a username for logging purposes << ")
        self.gathering_data = False

        # will be useful once data gathering automation is started
        self.manual_control = manual_control

        # keyboard stuff
        self.keyboard_input = None
        self.keyboard_impulse = False
        self.killswitch = False
        self.debug = True
        self.debug_window_process = True
        self.label = None

        # socket programming stuff
        self.server_data_receiver = DataBridgeServer_TCP(port_number=50000)
        print(f'Server Listening on port 50000')

        self.movement_data_sender = DataBridgeServer_TCP(port_number=50001)
        print(f'Server Listening on port 50001')

        # multithreading processes
        self.window_thread = threading.Thread(target=self.window_process)   # Pygame window process
        self.window_thread.start() # Thread 1
        self.listener_thread = threading.Thread(target=self.kb_listener)    # Keyboard listener
        self.listener_thread.start() # Thread 2
        self.data_listener_thread = threading.Thread(target=self.super_json_listener)
        self.data_listener_thread.start() # Thread 3

    def window_process(self):

        pygame.init()
        self.screen = pygame.display.set_mode((300, 300))
        pygame.display.set_caption('Turtlebot Controller')
        self.pygame_font = pygame.font.SysFont('Arial', 20)
        self.text_color = (25, 25, 25)
        self.text_display_content = 'Enter prompt in Python Terminal.'
        self.is_collecting_data = False

        # Cap the frame rate
        pygame.time.Clock().tick(60)

        while not self.killswitch:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.killswitch = True
                    self.send_data('@KILL')
                    print('Killing program...')
                    pygame.quit()
                    sys.exit()
                elif event.type == pygame.KEYDOWN:
                    self.keyboard_input = event.unicode
                    self.keyboard_impulse = True
                    if self.debug_window_process:
                        print(f"pressed {self.keyboard_input}")
                elif event.type == pygame.KEYUP:
                    self.keyboard_input = None
                    self.keyboard_impulse = True
                    if self.debug_window_process:
                        print("Released key.")

            # Fill the screen with a background color
            self.screen.fill((255, 255, 255))

            # Render and display the text input
            text_surface = self.pygame_font.render(
                self.text_display_content, True, self.text_color)
            self.screen.blit(text_surface, (10, 10))

            # Update the display
            pygame.display.flip()

    def super_json_listener(self):

        t = time.time() + 1.0
        x = 0
        while True:
            
            if not self.gathering_data: time.sleep(0.007); continue
            if time.time() > t: 
                t = time.time() + 1.0
                x = 0
            x += 1
            data = json.loads(self.server_data_receiver.receive_data().decode())
            if self.data_buffer == None: print("WARNING: data buffer is still None type."); continue
            
            if args.display: # set to True to enable opencv camera
                camera_frame = data['frame_data']
                encoded_data = base64.b64decode(camera_frame)

                # Convert the bytes to a numpy array
                nparr = np.frombuffer(encoded_data, np.uint8)

                # Decode the numpy array to an OpenCV image
                frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                cv2.imshow('frame',frame)
                cv2.waitKey(1)
                print(x)

            self.data_buffer.append(data)

    def prompt_generator(self):

        prompt_type = ['move', 'rotate', 'two_inst']
        selected = random.sample(prompt_type, 1)
    
        if selected[0] == 'move':
            random_number = random.randint(1, 30)
            random_number = random_number / 10  
            prompt = "Move forward by " + str(random_number) + " meters"
    
        elif selected[0] == 'rotate':
            direction = ['left', 'right']
            selected_direction = random.sample(direction, 1)
            prompt = "Rotate " + selected_direction[0] + " by " + str(random.randrange(10,365,5)) + " degrees"
    
        elif selected[0] == 'two_inst':
            random_number = random.randint(1, 30)
            random_number = random_number / 10  
            prompt_a = str(random_number) + " meters"
    
            direction = ['left', 'right']
            selected_direction = random.sample(direction, 1)
            prompt_b = selected_direction[0] + " by " + str(random.randrange(10,365,5)) + " degrees"
    
            variation = [1, 2]
            selected_variation = random.sample(variation, 1)
            if selected_variation[0] == 1:
                prompt = "Move forward by " + prompt_a + " then rotate " + prompt_b
            elif selected_variation[0] == 2:
                prompt = "Rotate " + prompt_b + " then move forward by " + prompt_a
    
        return prompt

    def kb_listener(self):  # do Turtlebot controller stuff here

        """
        Instructions: // all data instructions are 5 bytes wide
        @XXXX, where the 4 X's are the instructions to be sent to the robot.
        @0000 - Stop
        @FRWD - Forward
        @LEFT - Turn Left
        @RGHT - Turn Right
        @STRT - start recording data
        @STOP - stop recording data
        @KILL - stop program
        @RNDM - randomize position, -5 <= x,y <= 5, 0 <= theta <= 2pi # NOTE: may not be used for now

        Each data send starts with sending an 8-byte long size indicator. Each data segment is sent in 1024-byte-sized chunks.
        Once all is received, receiver sends an 'OK' to sender to indicate that is done processing whatever data was
        sent over.
        """

        # start host loop -> enter prompt -> start logging -> do stuff -> end logging -> generate unique id -> confirm save -> save data as a json with unique id

    
        # assumption: socket connection is successful
        while not self.killswitch:  # Start host loop

            prompt = prompt_generator()                             # random_generated
            print(prompt)
            if prompt != '$CONTROL':
                print('sending START')
                self.data_buffer = []                               # reset buffer. this will be filled up somewhere else (self.super_json_listener)
                self.gathering_data = True
                self.movement_data_sender.send_data('@STRT')        # send data gathering start signal
                print('waiting for CONT')
                self.movement_data_sender.receive_data()            # wait for @CONT
            else:
                print('Activating movement debug mode.')

            while True: # Inner loop, data collection
            
                if not self.keyboard_impulse: time.sleep(0.01667); continue
                print("input received:", self.keyboard_input)

                if self.keyboard_input == 'w': 
                    self.movement_data_sender.send_data(b'@FRWD')
                    self.keyboard_impulse = False                   # set to False to be able to tell when user lets go of the key.

                elif self.keyboard_input == 'a':
                    self.movement_data_sender.send_data(b'@LEFT')
                    self.keyboard_impulse = False

                elif self.keyboard_input == 'd':
                    self.movement_data_sender.send_data(b'@RGHT')
                    self.keyboard_impulse = False

                elif self.keyboard_input == None:                   # Key was let go
                    self.movement_data_sender.send_data(b'@0000')
                    self.keyboard_impulse = False

                elif self.keyboard_input == ']':
                    self.movement_data_sender.send_data(b'@STOP')
                    self.keyboard_impulse = False
                    print("Data collection is finished for this iteration.")
                    break

                time.sleep(0.01667)

            # confirm if data is good to be saved
            self.gathering_data = False

            if prompt == '$CONTROL':
                continue

            while True:
                confirmation = input("Save log? (y/n) << ")
                if confirmation == 'y' or confirmation == 'n': break

            if confirmation == 'y':

                # save data into a buffer
                
                json_file = {
                            "username":self.current_user, "natural_language_prompt":prompt,
                            "timestamp_s":time.ctime(), "timestamp_float":time.time(),
                            "states":self.data_buffer
                            }
                
                fname = self.generate_random_filename()

                with open(os.path.join("datalogs",fname),'w') as f:
                    f.write(json.dumps(json_file, indent=4))
            
                print("Instance saved.")
                
            else:
                print("Instance removed.")

    def generate_random_filename(self):
        random_filename = str(uuid.uuid4().hex)[:16]
        return random_filename

    def send_data(self, data: bytes):

        if isinstance(data, str):
            data = data.encode()

        # NOTE: data may or may not be in string format.

        length_bytes = struct.pack('!I', len(data))

        if self.debug:
            print('[S] Sending byte length...')
        self.client.sendall(length_bytes)
        ack = self.client.recv(2)  # wait for other side to process data size
        if ack != b'OK':
            print(f'[S] ERROR: unmatched send ACK. Received: {ack}')
        if self.debug:
            print('[S] ACK good')

        if self.debug:
            print('[S] Sending data...')
        self.client.sendall(data)  # send data
        if self.debug:
            print('[S] Data sent; waiting for ACK...')
        ack = self.client.recv(2)  # wait for other side to process data size
        if ack != b'OK':
            print(f'[S] ERROR: unmatched send ACK. Received: {ack}')
        if self.debug:
            print('[S] ACK good. Data send success.')

    def receive_data(self):

        # NOTE: Returns data in BINARY. You must decode it on your own

        if self.debug:
            print('[R] Waiting for byte length...')
        length_bytes = self.client.recv(4)
        length = struct.unpack('!I', length_bytes)[0]
        if self.debug:
            print(f'[R] Byte length received. Expecting: {length}')
        data, data_size = b'', 0

        self.client.send(b'OK')  # allow other side to send over the data
        if self.debug:
            print(f'[R] ACK sent.')
        while data_size < length:

            chunk_size = min(2048, length - data_size)
            data_size += chunk_size
            data += self.client.recv(chunk_size)
            if self.debug:
                print(f'[R] RECV {chunk_size}')

        if self.debug:
            print('[R] Transmission received successfull. Sending ACK')
        self.client.send(b'OK')  # unblock other end
        if self.debug:
            print('[R] ACK sent.')
        return data  # up to user to interpret the data


x = turtlebot_controller()
while not x.killswitch:
    time.sleep(5)
