import cv2
import numpy as np
import pygame.image

from gfs.gui.interface import Interface
from gfs.pallet import IVORY


class MainView:
    def __init__(self, width, height, session):
        self.surface_configuration = (width, height)
        self.next_state = None
        self.session = session

        self.camera_image_subscriber = self.session.declare_subscriber("turtle/camera", self.camera_image_callback)
        self.camera_image = None

        self.interface = Interface()

    def quit(self):
        self.camera_image_subscriber.undeclare()

    def camera_image_callback(self, sample):
        npImage = np.frombuffer(bytes(sample.value.payload), dtype=np.uint8)
        matImage = cv2.imdecode(npImage, 1)

        self.camera_image = pygame.surfarray.make_surface(matImage)

    def keyboard_input(self, event):
        self.interface.keyboard_input(event)

    def mouse_input(self, event):
        self.interface.mouse_input(event)

    def mouse_motion(self, event):
        self.interface.mouse_motion(event)

    def update(self):
        self.interface.update()

    def render(self, surface):
        surface.fill(IVORY)

        if self.camera_image is not None:
            surface.blit(self.camera_image, 0, 0)

        self.interface.render(surface)
