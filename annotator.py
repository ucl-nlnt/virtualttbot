# imports
from pygame.locals import *
import pygame.camera
import pygame
import datetime
import pygame_gui
from pygame_gui.core import ObjectID
from pygame_gui.elements import UIButton, UITextEntryBox, UILabel, UIImage, UITextEntryLine
import json

# config variables
screen_width = 1280
screen_height = 720
screen_margins = 25
background_color = '#000000'

button_width = 100
button_height = 50

# pygame setup
pygame.init()


pygame.display.set_caption("NLNT Turtlebot Data Annotator Studio")
icon_image = pygame.image.load("media/annotator-icon.png")
pygame.display.set_icon(icon_image)

screen = pygame.display.set_mode((screen_width, screen_height))
background = pygame.Surface((screen_width, screen_height))
background.fill(pygame.Color(background_color))
manager = pygame_gui.UIManager((screen_width, screen_height), "theme.json")
clock = pygame.time.Clock()

# initiate camera
pygame.camera.init()
camlist = pygame.camera.list_cameras()
if not camlist:
    print("ERROR: No cameras detected")
camera_index = 1  # If your device has multiple cameras, adjust to the correct one
cam = pygame.camera.Camera(camlist[camera_index], (1280, 720))
cam.start()
image = cam.get_image()

# state variables setup
is_running = True
""" (bool): if the program is running or not """
is_recording = True
""" (bool): if the recorded data is saved to the output or not """
time_delta = 0
tick_counter = 0

# global variables
sensor_text = ""

# UI elements
start_button = UIButton(relative_rect=pygame.Rect((screen_width-(screen_margins+button_width)*4, screen_height-screen_margins-button_height), (button_width, button_height)),
                        text='Start',
                        manager=manager, object_id=ObjectID(class_id='@rounded_buttons',
                                                            object_id='#start_button'), tool_tip_text="start recording data")

stop_button = UIButton(relative_rect=pygame.Rect((screen_width-(screen_margins+button_width)*3, screen_height-screen_margins-button_height), (button_width, button_height)),
                       text='Stop',
                       manager=manager, object_id=ObjectID(class_id='@rounded_buttons',
                                                           object_id='#stop_button'), tool_tip_text="stop recording data")

clear_button = UIButton(relative_rect=pygame.Rect((screen_width-(screen_margins+button_width)*2, screen_height-screen_margins-button_height), (button_width, button_height)),
                        text='Clear',
                        manager=manager, object_id=ObjectID(class_id='@rounded_buttons',
                                                            object_id='#clear_button'), tool_tip_text="clear input field")

save_button = UIButton(relative_rect=pygame.Rect((screen_width-(screen_margins+button_width), screen_height-screen_margins-button_height), (button_width, button_height)),
                       text='Save',
                       manager=manager, object_id=ObjectID(class_id='@rounded_buttons',
                                                           object_id='#save_button'), tool_tip_text="save annotated data to output file")

data_text = UITextEntryBox(relative_rect=pygame.Rect((652.5, 275), (602.5, 350)),
                           initial_text=sensor_text,
                           manager=manager)

image_box = UIImage(relative_rect=pygame.Rect((25, 275), (602.5, 350)),
                    image_surface=image,
                    manager=manager,)

prompt_box = UITextEntryLine(relative_rect=pygame.Rect((25, 175), (1230, 75)),
                             placeholder_text="Enter prompt here...",
                             manager=manager,)


user_box = UITextEntryLine(relative_rect=pygame.Rect((25, 100), (150, 50)),
                           placeholder_text="username",
                           manager=manager,)

instance_box = UITextEntryLine(relative_rect=pygame.Rect((200, 100), (150, 50)),
                               placeholder_text="instance_id",
                               manager=manager,)

connection_text = UILabel(relative_rect=pygame.Rect((25, 25), (300, 50)),
                          text="Status: CONNECTED Ping: 20 ms",
                          manager=manager, object_id=ObjectID(class_id='@text_input',
                                                              object_id='#connection_text'),)


battery_text = UILabel(relative_rect=pygame.Rect((725, 25), (125, 50)),
                       text="Battery: 100%",
                       manager=manager, object_id=ObjectID(class_id='@text_input',
                                                           object_id='#battery_text'),)

time_text = UILabel(relative_rect=pygame.Rect((1075, 25), (200, 50)),
                    text="Time:",
                    manager=manager, object_id=ObjectID(class_id='@text_input',
                                                        object_id='#battery_text'),)

while is_running:
    time_delta = clock.tick(60) / 1000  # limits FPS to 60

    # update every tick
    if (tick_counter > 0):
        current_time = datetime.datetime.now()
        unix_timestamp = int(current_time.timestamp())
        time_text.set_text(f'Time: {unix_timestamp}')

    # update image output every 2 ticks (30 fps video feed at 60 fps game output)
    if (tick_counter % 2 == 0):
        tick_counter = 0
        image = cam.get_image()
        image_box.set_image(image)

    # reset tick every 60
    if (tick_counter == 59):
        tick_counter = 0

    # event handler
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            is_running = False

        if event.type == pygame_gui.UI_BUTTON_PRESSED:
            if event.ui_element == start_button:
                print('[events/buttonPressed]: start button pressed!')
                is_recording = True
            elif event.ui_element == stop_button:
                print('[events/buttonPressed]: stop button pressed!')
                is_recording = False
            elif event.ui_element == clear_button:
                print('[events/buttonPressed]: clear button pressed!')
                user_box.set_text("")
                instance_box.set_text("")
                data_text.set_text("")
                is_recording = False
            elif event.ui_element == save_button:
                print('[events/buttonPressed]: save button pressed!')
                username = user_box.get_text()
                instance_id = instance_box.get_text()
                prompt = prompt_box.get_text()
                sensor_data = data_text.get_text()

                new_data = {"timestamp": unix_timestamp,
                            "username": username, "instance_id": instance_id, "prompt": prompt, "sensor_data": sensor_data}

                try:
                    with open('annotator-output.txt', 'r') as file:
                        existing_data = file.read()
                except FileNotFoundError:
                    existing_data = ""

                output = existing_data + '\n' + \
                    json.dumps(new_data, separators=(",", ":"))

                with open('annotator-output.txt', 'w') as file:
                    json.dump(output, file)

                data_text.set_text("")
                is_recording = False

        manager.process_events(event)

    manager.update(time_delta)

    screen.blit(background, (0, 0))
    manager.draw_ui(screen)

    pygame.display.update()
    tick_counter += 1


pygame.quit()
