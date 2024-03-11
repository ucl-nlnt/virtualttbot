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


if not os.path.exists("datalogs"):
    os.mkdir("datalogs")

# config variables
screen_width = 1280
screen_height = 720
screen_margins = 25
background_color = '#000000'

button_width = 100
button_height = 50


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

        # USE IN MACOS
        self.showInterface()

        # USE IN UBUNTU
        # self.window_thread = threading.Thread(
        #     target=self.showInterface)
        # self.window_thread.start()
        print(f'User interface initiated')

        self.server_data_receiver = DataBridgeServer_TCP(port_number=50000)
        print(f'Server Listening on port 50000')

        self.movement_data_sender = DataBridgeServer_TCP(port_number=50001)
        print(f'Server Listening on port 50001')

        self.data_listener_thread = threading.Thread(
            target=self.super_json_listener)
        self.data_listener_thread.start()

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
            (screen_width, screen_height), "theme.json")
        self.clock = self.pygame.time.Clock()

        # initiate camera
        self.pygame.camera.init()
        camlist = self.pygame.camera.list_cameras()
        if not camlist:
            print("ERROR: No cameras detected")
        camera_index = 1  # If your device has multiple cameras, adjust to the correct one
        self.cam = self.pygame.camera.Camera(
            camlist[camera_index], (1280, 720))
        self.cam.start()
        self.image = self.cam.get_image()

        # PYGAME INIT END

        self.initializeElements()

        # main loop
        while self.is_running:
            time_delta = self.clock.tick(60) / 1000  # limits FPS to 60

            # update every tick
            if (self.tick_counter > 0):
                current_time = datetime.datetime.now()
                unix_timestamp = int(current_time.timestamp())
                self.time_label.set_text(f'Time: {unix_timestamp}')
                self.data_text.set_text(self.sensor_text)
                self.connection_label.set_text(self.ping_text)
                self.battery_label.set_text(self.battery_text)

            # update image output every 2 ticks (30 fps video feed at 60 fps game output)
            if (self.tick_counter > 0):
                self.tick_counter = 0
                image = self.cam.get_image()
                self.image_box.set_image(image)

            # reset tick every 60
            if (self.tick_counter == 59):
                self.tick_counter = 0

            # event handler
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.is_running = False

                if event.type == pygame.KEYDOWN:
                    if event.scancode == 79:
                        # RIGHT
                        print('[events/keyPressed]: d key pressed!')
                    if event.scancode == 80:
                        # LEFT
                        print('[events/keyPressed]: a key pressed!')
                    if event.scancode == 81:
                        # DOWN
                        print('[events/keyPressed]: s key pressed!')
                    if event.scancode == 82:
                        # UP
                        print('[events/keyPressed]: d key pressed!')

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

            pygame.display.update()
            self.tick_counter += 1

        self.pygame.quit()


annotator = Annotator()
