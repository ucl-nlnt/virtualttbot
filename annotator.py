# imports
from pygame.locals import *
import pygame.camera
import pygame
import datetime
import pygame_gui

# config variables
WIDTH = 1280
HEIGHT = 720
BACKGROUND_COLOR = '#000000'

# pygame setup
pygame.camera.init()
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
background = pygame.Surface((WIDTH, HEIGHT))
background.fill(pygame.Color(BACKGROUND_COLOR))
pygame.display.set_caption("NLNT Turtlebot Data Annotator Studio")
manager = pygame_gui.UIManager((WIDTH, HEIGHT))
clock = pygame.time.Clock()

# state variables setup
is_running = True
""" (bool): if the program is running or not """
time_delta = 0

# global variables


# BUTTONS
hello_button = pygame_gui.elements.UIButton(relative_rect=pygame.Rect((350, 275), (100, 50)),
                                            text='Say Hello',
                                            manager=manager)


while is_running:
    time_delta = clock.tick(60) / 1000  # limits FPS to 60

    current_time = datetime.datetime.now()
    unix_timestamp = int(current_time.timestamp())

    # event handler
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            is_running = False

        if event.type == pygame_gui.UI_BUTTON_PRESSED:
            if event.ui_element == hello_button:
                print('Hello World!')

        manager.process_events(event)

    manager.update(time_delta)

    screen.blit(background, (0, 0))
    manager.draw_ui(screen)

    pygame.display.update()


pygame.quit()
