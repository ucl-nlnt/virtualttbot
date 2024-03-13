# imports
from pygame.locals import *
import pygame
import pygame.camera
import datetime
import pygame_gui
from pygame_gui.core import ObjectID
from pygame_gui.elements import UIButton, UITextEntryBox, UILabel, UIImage, UITextEntryLine
import json
import os
from KNetworking import DataBridgeServer_TCP
import threading
import time
import struct
import uuid
from simple_chalk import chalk
import pprintpp

if not os.path.exists("datalogs"):
    os.mkdir("datalogs")

# config variables
screen_width = 1280
screen_height = 720
screen_margins = 25
background_color = '#000000'

button_width = 100
button_height = 50

ANNOTATOR_DEBUG = False


class Annotator:
    def __init__(self, manual_control=True) -> None:
        self.data_buffer = None
        self.gathering_data = False

        # will be useful once data gathering autmation is started
        self.manual_control = manual_control

        # keyboard stuff
        self.keyboard_input = None
        self.keyboard_impulse = False
        self.killswitch = False
        self.debug = True
        self.debug_window_process = True
        self.label = None

        """boolean: if device has a working camera"""
        self.hasCamera = False

        # state variables setup
        self.is_running = True
        """ (bool): if the program is running or not """
        self.is_recording = True
        """ (bool): if the recorded data is saved to the output or not """
        self.time_delta = 0
        self.tick_counter = 0

        # global variables
        self.ping = 20
        """network latency between turtlebot and client PC in ms"""
        self.ping_text = f'Status: CONNECTED Ping: {self.ping} ms'
        battery = 80
        """battery capacity of the turtlebot"""
        self.battery_text = f'{battery}%'
        self.sensor_data = [{}]
        """enter sensor data into this dict list"""
        self.sensor_text = ""
        """enter sensor data into this string"""
        # TODO: save sensor data into the above global variables

        if not ANNOTATOR_DEBUG:
            self.server_data_receiver = DataBridgeServer_TCP(port_number=50000)
            print(f'Server Listening on port 50000')

            self.movement_data_sender = DataBridgeServer_TCP(port_number=50001)
            print(f'Server Listening on port 50001')

        self.window_thread = threading.Thread(
            target=self.showInterface)
        self.window_thread.start()
        print(f'User interface initiated')

        if not ANNOTATOR_DEBUG:
            self.data_listener_thread = threading.Thread(
                target=self.super_json_listener)
            self.data_listener_thread.start()

        # reset buffer. this will be filled up somewhere else (self.super_json_listener)
        self.data_buffer = []
        self.gathering_data = True
        
        if not ANNOTATOR_DEBUG:
            # send data gathering start signal
            self.movement_data_sender.send_data('@STRT')
            print('waiting for CONT')
            self.movement_data_sender.receive_data()            # wait for @CONT

    def super_json_listener(self):

        while True:

            if not self.gathering_data:
                time.sleep(0.007)
                continue
            data = self.server_data_receiver.receive_data().decode()
            if self.data_buffer == None:
                print("WARNING: data buffer is still None type.")
                continue
            self.data_buffer.append(data)

    def initializeElements(self):
        # UI elements
        self.start_button = UIButton(relative_rect=pygame.Rect((screen_width-(screen_margins+button_width)*4, screen_height-screen_margins-button_height), (button_width, button_height)),
                                     text='Start',
                                     manager=self.manager, object_id=ObjectID(class_id='@rounded_buttons',
                                                                              object_id='#start_button'), tool_tip_text="start recording data")

        self.stop_button = UIButton(relative_rect=pygame.Rect((screen_width-(screen_margins+button_width)*3, screen_height-screen_margins-button_height), (button_width, button_height)),
                                    text='Stop',
                                    manager=self.manager, object_id=ObjectID(class_id='@rounded_buttons',
                                                                             object_id='#stop_button'), tool_tip_text="stop recording data")

        self.clear_button = UIButton(relative_rect=pygame.Rect((screen_width-(screen_margins+button_width)*2, screen_height-screen_margins-button_height), (button_width, button_height)),
                                     text='Clear',
                                     manager=self.manager, object_id=ObjectID(class_id='@rounded_buttons',
                                                                              object_id='#clear_button'), tool_tip_text="clear input field")

        self.save_button = UIButton(relative_rect=pygame.Rect((screen_width-(screen_margins+button_width), screen_height-screen_margins-button_height), (button_width, button_height)),
                                    text='Save',
                                    manager=self.manager, object_id=ObjectID(class_id='@rounded_buttons',
                                                                             object_id='#save_button'), tool_tip_text="save annotated data to output file")

        self.data_text = UITextEntryBox(relative_rect=pygame.Rect((652.5, 275), (602.5, 350)),
                                        initial_text=self.sensor_text,
                                        manager=self.manager)

    
        self.image_box = UIImage(relative_rect=pygame.Rect((25, 275), (602.5, 350)),
                                 image_surface=self.image,
                                 manager=self.manager,)

        self.prompt_box = UITextEntryLine(relative_rect=pygame.Rect((25, 175), (1230, 75)),
                                          placeholder_text="Enter prompt here...",
                                          manager=self.manager,)

        self.user_box = UITextEntryLine(relative_rect=pygame.Rect((25, 100), (150, 50)),
                                        placeholder_text="username",
                                        manager=self.manager,)

        self.instance_box = UITextEntryLine(relative_rect=pygame.Rect((200, 100), (150, 50)),
                                            placeholder_text="instance_id",
                                            manager=self.manager,)

        self.connection_label = UILabel(relative_rect=pygame.Rect((25, 25), (300, 50)),
                                        text=self.ping_text,
                                        manager=self.manager, object_id=ObjectID(class_id='@text_input',
                                                                                 object_id='#connection_text'),)

        self.battery_label = UILabel(relative_rect=pygame.Rect((725, 25), (125, 50)),
                                     text=self.battery_text,
                                     manager=self.manager, object_id=ObjectID(class_id='@text_input',
                                                                              object_id='#battery_text'),)

        self.time_label = UILabel(relative_rect=pygame.Rect((1075, 25), (200, 50)),
                                  text=f'Time: {int(datetime.datetime.now().timestamp())}',
                                  manager=self.manager, object_id=ObjectID(class_id='@text_input',
                                                                           object_id='#battery_text'))

        return 
    def showInterface(self):

        # PYGAME INIT START
        self.pygame = pygame
        self.pygame.init()

        # annotator app setup
        self.pygame.display.set_caption("NLNT Turtlebot Data Annotator Studio")
        icon_image = self.pygame.image.load("media/annotator-icon.png")
        self.pygame.display.set_icon(icon_image)

        # UI setup
        self.screen = self.pygame.display.set_mode(
            (screen_width, screen_height))
        self.background = self.pygame.Surface((screen_width, screen_height))
        self.background.fill(self.pygame.Color(background_color))
        self.manager = pygame_gui.UIManager(
            (screen_width, screen_height), "config/theme.json")
        self.clock = self.pygame.time.Clock()

        print(chalk.magenta.bold("Initiating cameras..."))

        # initiate camera
        self.pygame.camera.init()
        camlist = self.pygame.camera.list_cameras()
        
        if not camlist:
            print(chalk.red.bold("ERROR: No cameras detected"))
            self.hasCamera = False
            return
    
        print(chalk.green.bold("Cameras initiated"))
        self.hasCamera = True
        camera_index = 0 # If your device has multiple cameras, adjust to the correct one
        self.cam = self.pygame.camera.Camera(
            camlist[camera_index], (1280, 720))
        self.cam.start()
        self.image = self.cam.get_image()

        # PYGAME INIT END
        
        print(chalk.magenta.bold("Initiating elements..."))
        

        self.initializeElements()
        
        print(chalk.green.bold("Elements initiated"))

        self.is_key_pressed = False
        
       

        # main loop
        while self.is_running:
            print(self.tick_counter)
            time_delta = self.clock.tick(60) / 1000  # limits FPS to 60
            
            if (self.tick_counter % 5 == 0):
                if (len(self.sensor_text)>=1 and self.sensor_text[0]=="b"):
                    sensor_data = self.parseRawData(self.sensor_text)
                    print(sensor_data.keys())
                    print(sensor_data["odometry"])
            

            # update every tick
            if (self.tick_counter > 0):
                current_time = datetime.datetime.now()
                unix_timestamp = int(current_time.timestamp())
                self.time_label.set_text(f'Time: {unix_timestamp}')
                if ANNOTATOR_DEBUG:
                    self.sensor_text = "Lorem Ipsum"
                else:
                    self.sensor_text = self.data_buffer[-1] 
                    
                self.data_text.set_text(self.sensor_text)
                self.connection_label.set_text(self.ping_text)
                self.battery_label.set_text(self.battery_text)

            # update image output every 2 ticks (30 fps video feed at 60 fps game output)
            if (self.tick_counter %2 == 1):
                if self.hasCamera:
                    image = self.cam.get_image()
                    self.image_box.set_image(image)

            # reset tick every 60
            if (self.tick_counter >= 59):
                
                self.tick_counter = 0
                

            # event handler
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.is_running = False

                if event.type == pygame.KEYDOWN:
                    if event.scancode == 79:
                        # RIGHT
                        # print('[events/keyPressed]: d key pressed!')
                        self.movement_data_sender.send_data(b'@RGHT')
                        self.is_key_pressed = True
                    elif event.scancode == 80:
                        # LEFT
                        # print('[events/keyPressed]: a key pressed!')
                        self.movement_data_sender.send_data(b'@LEFT')
                        self.is_key_pressed = True
                    elif event.scancode == 81:
                        # DOWN
                        # print('[events/keyPressed]: s key pressed!')
                        self.movement_data_sender.send_data(b'@0000')
                    elif event.scancode == 82:
                        # UP
                        print('[events/keyPressed]: w key pressed!')
                        self.movement_data_sender.send_data(b'@FRWD')
                        self.is_key_pressed = True
                    else:
                        self.movement_data_sender.send_data(b'@0000')

                if event.type == pygame_gui.UI_BUTTON_PRESSED:
                    if event.ui_element == self.start_button:
                        print('[events/buttonPressed]: start button pressed!')
                        self.is_recording = True
                    elif event.ui_element == self.stop_button:
                        print('[events/buttonPressed]: stop button pressed!')
                        self.is_recording = False
                    elif event.ui_element == self.clear_button:
                        print('[events/buttonPressed]: clear button pressed!')
                        self.user_box.set_text("")
                        self.instance_box.set_text("")
                        self.data_text.set_text("")
                        self.is_recording = False
                    elif event.ui_element == self.save_button:
                        print('[events/buttonPressed]: save button pressed!')
                        username = self.user_box.get_text()
                        instance_id = self.instance_box.get_text()
                        prompt = self.prompt_box.get_text()
                        sensor_data = self.data_text.get_text()

                        image = self.cam.get_image()
                        pygame.image.save(
                            image, f"cam-output/{unix_timestamp}.png")
                        # TODO: update image saving format/ frequency

                        new_data = {"timestamp": unix_timestamp,
                                    "username": username, "instance_id": instance_id, "prompt": prompt, "sensor_data": sensor_data}

                        # TODO: update data format here depending on our needs

                        try:
                            with open('annotator-output.txt', 'r') as file:
                                existing_data = file.read()
                        except FileNotFoundError:
                            existing_data = ""

                        output = existing_data + '\n' + \
                            json.dumps(new_data, separators=(",", ":"))

                        with open('annotator-output.txt', 'w') as file:
                            file.write(output)

                        self.data_text.set_text("")
                        self.is_recording = False

                self.manager.process_events(event)


            self.manager.update(time_delta)
            self.screen.blit(self.background, (0, 0))
            self.manager.draw_ui(self.screen)

            self.tick_counter = self.tick_counter + 1
            pygame.display.update()

        self.pygame.quit()

    def hasKeyPressed(self, dicts, field, value):
        for dictionary in dicts:
            if field in dictionary and dictionary[field] == value:
                return True
        return False

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

    def parseRawData(self,raw):
        a = raw[1:].replace("'",'"').replace('array("f",',"").replace("])","]").replace("None",'"None"').replace("(","[").replace(")","]")
        print(a)
        out = json.load(a)
        print(out)
        return out
annotator = Annotator()
