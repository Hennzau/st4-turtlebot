import cv2
import numpy as np
import pygame.image

from gfs.gui.interface import Interface
from gfs.gui.used import Used
from gfs.pallet import IVORY, DARKBLUE

from dataclasses import dataclass
from pycdr2 import IdlStruct
from pycdr2.types import int8, int32, uint32, float64


@dataclass
class Vector3(IdlStruct, typename="Vector3"):
    x: float64
    y: float64
    z: float64


@dataclass
class Twist(IdlStruct, typename="Twist"):
    linear: Vector3
    angular: Vector3


def calculate_twist(linear, angular):
    t = Twist(linear=Vector3(x=linear, y=0.0, z=0.0),
              angular=Vector3(x=0.0, y=0.0, z=angular))

    return t.serialize()


class MainView:
    def __init__(self, width, height, session):
        self.surface_configuration = (width, height)
        self.next_state = None
        self.session = session

        self.camera_image_subscriber = self.session.declare_subscriber("turtle/camera", self.camera_image_callback)
        self.camera_image = None

        self.cmd_vel_publisher = self.session.declare_publisher("turtle/cmd_vel")

        self.interface = Interface()
        self.interface.add_gui(Used(pygame.K_UP, "↑", (200, 500), self.turtle_up))
        self.interface.add_gui(Used(pygame.K_DOWN, "↓", (200, 550), self.turtle_down))
        self.interface.add_gui(Used(pygame.K_LEFT, "←", (175, 525), self.turtle_left))
        self.interface.add_gui(Used(pygame.K_RIGHT, "→", (225, 525), self.turtle_right))

    def quit(self):
        self.camera_image_subscriber.undeclare()

    def camera_image_callback(self, sample):
        image = np.frombuffer(bytes(sample.value.payload), dtype=np.uint8)
        image = cv2.imdecode(image, 1)
        image = np.rot90(image)

        self.camera_image = pygame.surfarray.make_surface(image)

    def turtle_up(self):
        twist = calculate_twist(20.0, 0.0)
        self.cmd_vel_publisher.put(twist)

    def turtle_down(self):
        twist = calculate_twist(-20.0, 0.0)
        self.cmd_vel_publisher.put(twist)

    def turtle_left(self):
        twist = calculate_twist(0.0, 150.0)
        self.cmd_vel_publisher.put(twist)

    def turtle_right(self):
        twist = calculate_twist(0.0, -150.0)
        self.cmd_vel_publisher.put(twist)

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
            surface.draw_rect(DARKBLUE, pygame.Rect(10, 10, self.camera_image.get_width() + 10,
                                                    self.camera_image.get_height() + 10))
            surface.blit(self.camera_image, 15, 15)

        self.interface.render(surface)
