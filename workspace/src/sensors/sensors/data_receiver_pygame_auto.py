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
import random
import lzma

from KNetworking import DataBridgeServer_TCP
from prompt_randomizer import prompt_randomizer
from csv_randomizer import random_csv
from l3_csv_randomizer import l3_prompt_randomizer
from new_prompt_randomizer import prompt_maker

if not os.path.exists("datalogs"):
    os.mkdir("datalogs")

parser = argparse.ArgumentParser(description="Turtlebot3 NLNT terminal-based controller.")

parser.add_argument("--enable_autorandomizer_from_csv", type=int, default=0, help="Creates a level 1 or 2 prompt based on a provided CSV file.")
parser.add_argument("--csv_path",type=str, default="nlnt_prompts/level2_rephrases.csv", help="Specifies path to NLNT natural language label dataset.")
parser.add_argument("--rotate_r_by",type=int, default=1, help="Rotate NLNT image by some amount before saving. Measured in Clockwise rotations.")
parser.add_argument("--name", default=None, help="Username")
parser.add_argument("--webcam", default=2, type=int, help="Selects webcam for external data gathering. Set to -1 to disable.")
parser.add_argument("--devmode", type=int, default=0, help="Activate developer mode.")
parser.add_argument("--webcam_h", type=int, default=720, help="Sets webcam feed height. Defaults to 720p resolution 16x9 aspect ratio.")
parser.add_argument("--webcam_w", type=int, default=1280, help="Sets webcam feed width. Defaults to 720p resolution 16x9 aspect ratio.")

parser.add_argument("--view_webcam", type=int, default=1, help="On by default, set to zero to disable.")
parser.add_argument("--view_raspi_cam", type=int, default=1, help="On by default, set to zero to disable.")

parser.add_argument("--l3_s_obj", type=int, default=0, help="Off by default, set to one to use l3_simple_object.csv.")
parser.add_argument("--l3_d_obj", type=int, default=0, help="Off by default, set to one to use l3_described_object.csv.")
parser.add_argument("--l3_stand", type=int, default=0, help="Off by default, set to one to use l3_stand_on_x.csv.")
parser.add_argument("--l3_ffw", type=int, default=0, help="Off by default, set to one to use l3_following.csv.")
parser.add_argument("--l3_all", type=int, default=0, help="Off by default, set to one to use l3_following.csv.")
parser.add_argument("--l12", type=int, default=0, help="Off by default, set to one to use l3_following.csv.")

args = parser.parse_args()
print(args)

l3 = args.l3_s_obj + args.l3_d_obj + args.l3_stand + args.l3_ffw + args.l3_all
l3_prompt = l3_prompt_randomizer(args.l3_s_obj, args.l3_d_obj, args.l3_stand, args.l3_ffw)

def retrieve_camera_indexes(): # used to check available cameras.

    max_tested = 10
    num_cameras = 0
    usable_camera_indexes = []

    for i in range(max_tested):
        cap = cv2.VideoCapture(i)
        if cap is None or not cap.isOpened():
            cap.release()
            continue
        else:
            usable_camera_indexes.append(i)
        num_cameras += 1
        cap.release()
        
    return usable_camera_indexes

class turtlebot_controller:

    def __init__(self, manual_control=True):

        self.data_buffer = None
        
        if args.name == None:
            self.current_user = input("enter a username for logging purposes << ")
        else:
            self.current_user = args.name

        self.gathering_data = False

        # will be useful once data gathering automation is started
        self.manual_control = manual_control

        self.sesh_count = 1

        # display bools
        self.display_webcam = args.view_webcam
        self.display_raspi_cam = args.view_raspi_cam

        # keyboard stuff
        self.keyboard_input = None
        self.keyboard_impulse = False
        self.killswitch = False
        self.debug = True
        self.debug_window_process = True
        self.label = None

        self.new_frame_impulse = False
        self.increment_twist_message = None # allows for robot turnability
        self.exempt_increment = False

        self.most_recent_webcam_frame = None
        self.most_recent_webcam_frame_base64 = None

        # socket programming stuff
        self.server_data_receiver = DataBridgeServer_TCP(port_number=50000)
        print(f'Server Listening on port 50000')

        self.movement_data_sender = DataBridgeServer_TCP(port_number=50001)
        print(f'Server Listening on port 50001')

        self.latest_raspi_camera_frame = None
        self.latest_webcam_camera_frame = None

        if args.webcam >= 0:

            available_cameras = retrieve_camera_indexes()

            if args.webcam not in available_cameras:
                
                print(available_cameras)
                cam_index = input("Error: Your currently-selected webcam is not working. Please select from those listed above (number only): ")
            
            else:
                cam_index = args.webcam

            self.webcam_object = cv2.VideoCapture(cam_index)
            self.webcam_object.set(cv2.CAP_PROP_FRAME_WIDTH, args.webcam_w)
            self.webcam_object.set(cv2.CAP_PROP_FRAME_HEIGHT, args.webcam_h)
            #self.webcam_object.set(cv2.CAP_PROP_AUTOFOCUS, 1) # enable autofocus if it supports it
            self.webcam_object.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1) # enable auto exposure

            self.usb_webcam_thread = threading.Thread(target=self.usb_webcam)
            self.usb_webcam_thread.start()

        if args.view_webcam or args.view_raspi_cam:

            self.display_thread = threading.Thread(target=self.disp_thread)
            self.display_thread.start()

        # multithreading processes
        self.window_thread = threading.Thread(target=self.window_process)   # Pygame window process
        self.window_thread.start() # Thread 1
        self.listener_thread = threading.Thread(target=self.kb_listener)    # Keyboard listener
        self.listener_thread.start() # Thread 2
        self.data_listener_thread = threading.Thread(target=self.super_json_listener)
        self.data_listener_thread.start() # Thread 3
        
    def disp_thread(self):

        while True:
            
            if not self.new_frame_impulse:
                time.sleep(0.05)
                continue

            print("raspi type",type(self.latest_raspi_camera_frame))
            print("webcam type",type(self.latest_webcam_camera_frame))

            if self.display_raspi_cam and isinstance(self.latest_raspi_camera_frame, np.ndarray):

                try:
                    cv2.imshow('Raspi (0.5 scale)', self.latest_raspi_camera_frame)

                except Exception as e:
                    print('could not show raspi camera', e)
                    self.display_raspi_cam = False

            if self.display_webcam and isinstance(self.latest_webcam_camera_frame, np.ndarray):
                
                try:
                    cv2.imshow('Webcam (0.5 scale)', self.latest_webcam_camera_frame)
                except Exception as e:
                    print('could not show webcam',e)
                    self.display_webcam = False

            self.new_frame_impulse = False
            cv2.waitKey(1)

    def calculate_sharpness(self, cv2_image):

        gray = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2GRAY)
        laplacian = cv2.laplacian(gray, cv2.CV_64F)
        sharpness = laplacian.var()
        return sharpness

    def usb_webcam(self):

        # initialize camera here.
        while True:
            
            ret, frame = self.webcam_object.read()

            if not ret:
                continue

            #sharpness = self.calculate_sharpness(frame)

            success, encoded_image = cv2.imencode('.jpg',frame, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
            if not success:
                continue

            width = int(frame.shape[1] * 0.5)
            height = int(frame.shape[0] * 0.5)
            new_dimensions = (width, height)

            frame = cv2.resize(frame, new_dimensions,interpolation = cv2.INTER_AREA)

            self.latest_webcam_camera_frame = frame
            self.most_recent_webcam_frame_base64 = base64.b64encode(encoded_image.tobytes()).decode('utf-8')

    def window_process(self):

        pygame.init()
        self.screen = pygame.display.set_mode((600, 600))
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

                    self.keyboard_impulse = True
                    keys = pygame.key.get_pressed()

                    if keys[pygame.K_w] and keys[pygame.K_d]:
                        
                        self.keyboard_input = "wd"

                    elif keys[pygame.K_w] and keys[pygame.K_a]:

                        self.keyboard_input = "wa"

                    elif keys[pygame.K_KP8]:

                        self.keyboard_input = 'kp8'

                    elif keys[pygame.K_KP7]:

                        self.keyboard_input = 'kp7'

                    elif keys[pygame.K_KP4]:

                        self.keyboard_input = 'kp4'

                    elif keys[pygame.K_w]:

                        self.keyboard_input = "w"

                    elif keys[pygame.K_d]:

                        self.keyboard_input = 'd'

                    elif keys[pygame.K_a]:

                        self.keyboard_input = 'a'

                    elif keys[pygame.K_RIGHTBRACKET]:

                        self.keyboard_input = ']'

                    elif keys[pygame.K_COMMA]:
                        self.keyboard_input = ','

                    elif keys[pygame.K_F10]:

                        self.keyboard_input = 'F10'

                    elif keys[pygame.K_KP_PLUS]:
                        
                        self.increment_twist_message = 0.3
                        self.exempt_increment = True
                        self.keyboard_impulse = True

                    elif keys[pygame.K_KP_MINUS]:

                        self.increment_twist_message = -0.3
                        self.exempt_increment = True
                        self.keyboard_impulse = True

                elif event.type == pygame.KEYUP:
                
                    if not self.exempt_increment:
                        self.keyboard_input = None
                        self.keyboard_impulse = True
                    else:
                        continue
                
            # Fill the screen with a background color
            self.screen.fill((255, 255, 255))

            # Render and display the text input
            text_surface = self.pygame_font.render(
                self.text_display_content, True, self.text_color)
            self.screen.blit(text_surface, (10, 10))

            # Update the display
            pygame.display.flip()

    def format_raspi_image(self, camera_frame: str, data):

        encoded_data = base64.b64decode(camera_frame)

        # Convert the bytes to a numpy array
        nparr = np.frombuffer(encoded_data, np.uint8)

        # Decode the numpy array to an OpenCV image
        frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

        # Frame Rotator
        if args.rotate_r_by:

            h, w = frame.shape[:2]
            angle = -90 * args.rotate_r_by
            rot_matrix = cv2.getRotationMatrix2D((w/2, h/2), angle, 1)
            frame = cv2.warpAffine(frame, rot_matrix, (w, h))

            # Re-encode the rotated frame to a format (e.g., JPEG) before converting it to base64
            retval, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
            
            print("TTB imencode:", retval)

            frame_data_as_string = base64.b64encode(buffer).decode('utf-8')

            # Update 'frame_data' in the JSON data structure
            data['frame_data'] = frame_data_as_string

            # scaling 50 percent image to make sure that it fits on the
            # laptop screen
            width = int(frame.shape[1] * 0.5)
            height = int(frame.shape[0] * 0.5)
            new_dimensions = (width, height)

            frame = cv2.resize(frame, new_dimensions,interpolation = cv2.INTER_AREA)
        
            self.latest_raspi_camera_frame = frame
            print('Turtlebot frame was received.')
            self.new_frame_impulse = True

    def super_json_listener(self):

        t = time.time() + 1.0
        x = 0

        while True:
            
            if not self.gathering_data: time.sleep(0.007); continue

            if time.time() > t:  # used to calculate framerate for debug purposes
                t = time.time() + 1.0
                x = 0
            x += 1

            item = self.server_data_receiver.receive_data()

            if item == None:
                print("ERROR: super json is NoneType")
                break

            data = json.loads(item.decode())

            if self.data_buffer == None: print("WARNING: data buffer is still None type."); continue

            # Camera display
            if args.view_webcam or args.rotate_r_by:

                # Decode camera data
                camera_frame = data['frame_data']

                if camera_frame != None: self.format_raspi_image(camera_frame=camera_frame, data=data)

            if args.webcam > -1:

                # append webcam data to 'data' frame
                
                if data['frame_data'] != None:

                    self.new_frame_impulse = True
                    print("Webcam capture:",type(self.most_recent_webcam_frame_base64), len(self.most_recent_webcam_frame_base64))
                    data['webcam_data'] = self.most_recent_webcam_frame_base64

            else:

                data['webcam_data'] = None
            
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
            
            print("Prompt #", str(self.sesh_count))                             # count number of saved prompts in current session

            if l3:
                # Level 3 Random Generated Prompts
                prompt = l3_prompt.new_prompt()
                print("Random Prompt:", prompt)

            elif args.enable_autorandomizer_from_csv:
            	# Random Generated Prompts  
                if args.enable_autorandomizer_from_csv == 1:
                    random_prompt = prompt_randomizer.prompt_maker()                      # generates up to 5 consecutive unique actions
                    prompt = random_prompt[0]
                    print("Random Prompt:",prompt)
                	#print("Prompt Equivalent: ", str(random_prompt[1]))
                    print("Cumulative Equivalent: ", str(random_prompt[2]))
                elif args.enable_autorandomizer_from_csv == 2:
                    prompt = random_csv.csv_randomizer()
                    print("Random Prompt:", prompt[0])

                    #if ("LEFT" in prompt[1]) or ("RGHT" in prompt[1]):
                    #    remove

                    print("Prompt Equivalent: ", prompt[1])
                elif args.enable_autorandomizer_from_csv == 3:
                    prompt = self.prompt_generator()
                    print("Random Prompt:",prompt)
                elif args.enable_autorandomizer_from_csv == 4:
                    prompt = self.csv_randomizer()
                    print("Random Prompt:",prompt)

            elif args.l12:

                mk = prompt_maker()
                inst = mk.maker(add_flags=True)
                prompt = inst[0]
                cumulative = inst[1]
                prompt_level = 3
                print(f'Prompt: {prompt}')
                print(f'Ground Truth: {cumulative}')
                

            elif not args.devmode:

                prompt = input("Enter prompt << ")
                print(f'Prompt is: "{prompt}"')
                prompt_level_set = False
                while not prompt_level_set and prompt != '$CONTROL' and not args.devmode:
                    
                    try:
                        prompt_level = int(input("Input prompt level for documentation (1, 2, or 3) << "))
                        if prompt_level not in [1, 2, 3]:
                            print('Please input a valid level number.')
                        else:
                            prompt_level_set = True

                    except Exception as e:
                        print(e)
                        print('Please input a valid entry number.')
                
            else:
                prompt = "$CONTROL"

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
            
                if not self.keyboard_impulse: 

                    time.sleep(0.01667)
                    continue

                if self.increment_twist_message != None:

                    self.rad_changer.send_data(str(self.increment_twist_message).encode())
                    self.increment_twist_message = None

                if self.keyboard_input == 'wa' and not self.exempt_increment:

                    self.movement_data_sender.send_data(b'@NTWS') # north-west
                    self.keyboard_impulse = False

                elif self.keyboard_input == 'kp8' and not self.exempt_increment:

                    self.movement_data_sender.send_data(b'@ADGB')
                    self.keyboard_impulse = False

                elif self.keyboard_input == 'kp7' and not self.exempt_increment:

                    self.movement_data_sender.send_data(b'@ADAG')
                    self.keyboard_impulse = False

                elif self.keyboard_input == 'kp4' and not self.exempt_increment:

                    self.movement_data_sender.send_data(b'@ADDG')
                    self.keyboard_impulse = False

                elif self.keyboard_input == 'wd' and not self.exempt_increment:

                    self.movement_data_sender.send_data(b'@NTES') # north-east
                    self.keyboard_impulse = False

                elif self.keyboard_input == 'w' and not self.exempt_increment: 

                    self.movement_data_sender.send_data(b'@FRWD')
                    self.keyboard_impulse = False                   # set to False to be able to tell when user lets go of the key.

                elif self.keyboard_input == 'a' and not self.exempt_increment:

                    self.movement_data_sender.send_data(b'@LEFT')
                    self.keyboard_impulse = False

                elif self.keyboard_input == 'd' and not self.exempt_increment:

                    self.movement_data_sender.send_data(b'@RGHT')
                    self.keyboard_impulse = False

                elif self.keyboard_input == None and not self.exempt_increment:  # key was let go
            
                    self.movement_data_sender.send_data(b'@0000')
                    self.keyboard_impulse = False

                elif self.keyboard_input == ']' and not self.exempt_increment:

                    self.movement_data_sender.send_data(b'@STOP')
                    self.keyboard_impulse = False
                    print("Data collection is finished for this iteration.")
                    break

                elif self.keyboard_input == '-' and not self.exempt_increment:
                    self.movement_data_sender.send_data(b'@KILL')
                    self.keyboard_impulse = False
                    print('Sent termination signal to Turtlebot3.')
                    break

                elif self.keyboard_input == ',' and not self.exempt_increment:
                    self.movement_data_sender.send_data(b'@OBST')
                    self.keyboard_impulse = False
                    print('Starting Obstacle Avoidance..')
                    break

                time.sleep(0.01667)

            # confirm if data is good to be saved
            self.gathering_data = False

            if prompt == '$CONTROL':
                continue

            while True:
                
                confirmation = input("Save log? (y/n/e) << ")
                if confirmation == 'y' or confirmation == 'n': break
                elif confirmation == 'e':
                    prompt = input("Revised Prompt: ")

            if confirmation == 'y' or confirmation == 'e':

                # save data into a buffer

                print('=================================')
                
                print('saving:', prompt)

                json_file = {
                            "username":self.current_user, "natural_language_prompt": prompt,
                            "timestamp_s":time.ctime(), "timestamp_float":time.time(),
                            "states":self.data_buffer, "prompt_level" : prompt_level
                            }

                fname = self.generate_random_filename()
                fname = fname + ".compressed_lzma"

                with open(os.path.join("datalogs",fname),'wb') as f:
                    
                    f.write(lzma.compress(json.dumps(json_file).encode('utf-8')))
                    print("Instance saved.")

                # integrity test

                print('Checking data integrity...')

                is_good = True
                detected_keyframes = 0
                states_with_images = []
                with open(os.path.join('datalogs',fname),'rb') as f:

                    data_file = json.loads(lzma.decompress(f.read()))
                    states = data_file['states']

                    for i, state in enumerate(states):

                        if 'webcam_data' in state.keys():
                            if state['frame_data'] == None:
                                is_good = False
                                print('Raspi camera data is no good.')
                                break
                            if state['webcam_data'] == None:
                                is_good = False
                                print('Webcam data is not good.')
                                break

                            frame_data = base64.b64decode(state['frame_data'])
                            frame_data_arr = cv2.imdecode(np.frombuffer(frame_data, dtype=np.uint8), cv2.IMREAD_COLOR)

                            webcam_data = base64.b64decode(state['webcam_data'])
                            webcam_data_arr = cv2.imdecode(np.frombuffer(webcam_data, dtype=np.uint8), cv2.IMREAD_COLOR)
                            if np.all(webcam_data_arr == 0):
                                is_good = False
                                print('Webcam data is all zeros.')
                                break
                            
                            if np.all(frame_data_arr == 0):
                                is_good = False
                                print('Raspi data is all zeros.')
                                break

                            states_with_images.append(i)
                            detected_keyframes += 1

                # compare images to detect same image errors
                comparisons = []
                for i in states_with_images:
                    for j in states_with_images:
                        if i == j:
                            continue

                        comparison_instance = sorted([i,j])
                        if comparison_instance not in comparisons:
                            comparisons.append(comparison_instance)
                
                for l in comparisons:
                    
                    i1, i2 = l
                    frame_data1 = base64.b64decode(states[i1]['frame_data'])
                    frame_data_arr1 = cv2.imdecode(np.frombuffer(frame_data1, dtype=np.uint8), cv2.IMREAD_COLOR)
                    frame_data2 = base64.b64decode(states[i2]['frame_data'])
                    frame_data_arr2 = cv2.imdecode(np.frombuffer(frame_data2, dtype=np.uint8), cv2.IMREAD_COLOR)

                    if np.array_equal(frame_data_arr1, frame_data_arr2):
                        is_good = False
                        print('Error: Identical webcam frames detected.')
                        break
                
                    webcam_data1 = base64.b64decode(states[i1]['webcam_data'])
                    webcam_data_arr1 = cv2.imdecode(np.frombuffer(webcam_data1, dtype=np.uint8), cv2.IMREAD_COLOR)
                    webcam_data2 = base64.b64decode(states[i2]['webcam_data'])
                    webcam_data_arr2 = cv2.imdecode(np.frombuffer(webcam_data2, dtype=np.uint8), cv2.IMREAD_COLOR)

                    if np.array_equal(webcam_data_arr1, webcam_data_arr2):
                        is_good = False
                        print('Error: Identical raspi frames detected.')
                        break
                        
                if is_good:
                    self.sesh_count += 1
                    print(f'All good. Detected {detected_keyframes} keyframes.')
                else:
                    os.remove(os.path.join('datalogs', fname))
                    print(f"{fname} deleted from directory due to erroneous save.") 
            else:
                print("Instance removed.")

    def generate_random_filename(self):
        random_filename = str(uuid.uuid4().hex)[:16]
        return random_filename

    def csv_randomizer():
        prompt_list = []
        with open(args.csv_path, 'r') as file:
            prompt_list = file.read().split('\n')

        if args.enable_autorandomizer_from_csv:
            rnd_num = random.randint(0, len(prompt_list)-1)
            next_prompt = prompt_list[rnd_num]
            return next_prompt
        return

x = turtlebot_controller()
while not x.killswitch:
    time.sleep(5)
