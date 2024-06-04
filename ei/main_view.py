import cv2
import numpy as np
import pygame.image

from gfs.gui.interface import Interface
from gfs.gui.used import Used
from gfs.pallet import IVORY, DARKBLUE


def message_callback(sample):
    print("MESSAGE RECEIVED : {}".format(sample.payload))


class MainView:
    def __init__(self, width, height, session):
        self.surface_configuration = (width, height)
        self.next_state = None
        self.session = session

        self.qcd = cv2.QRCodeDetector()

        self.camera_image_subscriber = self.session.declare_subscriber("turtle/camera", self.camera_image_callback)
        self.camera_image = None

        self.cmd_vel_publisher = self.session.declare_publisher("turtle/cmd_vel")
        self.message_publisher = self.session.declare_publisher("turtle/debug_message")
        self.message_subscriber = self.session.declare_subscriber("turtle/debug_message", message_callback)

        self.interface = Interface()
        self.interface.add_gui(Used(pygame.K_UP, "↑", (200, 500), self.turtle_up, self.turtle_standby_up))
        self.interface.add_gui(Used(pygame.K_DOWN, "↓", (200, 550), self.turtle_down, self.turtle_standby_down))
        self.interface.add_gui(Used(pygame.K_LEFT, "←", (175, 525), self.turtle_left, self.turtle_standby_left))
        self.interface.add_gui(Used(pygame.K_RIGHT, "→", (225, 525), self.turtle_right, self.turtle_standby_right))

        self.last_points = []

    def quit(self):
        self.camera_image_subscriber.undeclare()
        self.cmd_vel_publisher.undeclare()
        self.message_publisher.undeclare()
        self.message_subscriber.undeclare()

    def camera_image_callback(self, sample):
        image = np.frombuffer(bytes(sample.value.payload), dtype=np.uint8)
        # image = np.rot90(image)
        image = cv2.imdecode(image, 1)

        ret_qr, decoded_info, points, _ = self.qcd.detectAndDecodeMulti(image)
        if points is not None:
            image = cv2.polylines(image, points.astype(int), True, (255, 0, 0), 3)

        self.camera_image = pygame.surfarray.make_surface(image)

    def turtle_up(self):
        self.cmd_vel_publisher.put(("Forward", 20.0))

    def turtle_down(self):
        self.cmd_vel_publisher.put(("Forward", -20.0))

    def turtle_left(self):
        self.cmd_vel_publisher.put(("Rotate", 150.0))

    def turtle_right(self):
        self.cmd_vel_publisher.put(("Rotate", -150.0))

    def turtle_standby_up(self):
        self.cmd_vel_publisher.put(("Forward", 0.0))

    def turtle_standby_down(self):
        self.cmd_vel_publisher.put(("Forward", 0.0))

    def turtle_standby_left(self):
        self.cmd_vel_publisher.put(("Rotate", 0.0))

    def turtle_standby_right(self):
        self.cmd_vel_publisher.put(("Rotate", 0.0))

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
