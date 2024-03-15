import pygame.camera
import pygame
from simple_chalk import chalk

class Helper:
    def __init__(self) -> None:
        self.pygame = pygame
        self.pygame.init()
        self.pygame.camera.init()
        self.camlist = self.pygame.camera.list_cameras()

    def getCamList(self):
        """
        returns the device's available and valid camera list
        """
        return self.camlist

    def getCam(self, id):
        """
        returns a camera from the device's cam list
        """
        return self.camlist[id]


helper = Helper()
list = helper.getCamList()

print(chalk.green('Select your webcam from this list:'))
for i in range(len(list)):
    print(f'{i} - {list[i]}')
