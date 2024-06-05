import cv2
import numpy as np
import pygame.image
import io
import cmath

from dataclasses import dataclass

from matplotlib import pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas

from gfs.gui.interface import Interface
from gfs.gui.used import Used
from gfs.pallet import IVORY, DARKBLUE
from gfs.fonts import MOTO_MANGUCODE_30, render_font

from pycdr2 import IdlStruct
from pycdr2.types import uint32, float32
from typing import List

from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import Laser


@dataclass
class Time(IdlStruct, typename="Time"):
    sec: uint32
    nsec: uint32


@dataclass
class Header(IdlStruct, typename="Header"):
    stamp: Time
    frame_id: str


@dataclass
class LaserScan(IdlStruct, typename="LaserScan"):
    header: Header
    angle_min: float32
    angle_max: float32
    angle_increment: float32
    time_increment: float32
    scan_time: float32
    range_min: float32
    range_max: float32
    ranges: List[float32]
    intensities: List[float32]


def message_callback(sample):
    print("MESSAGE RECEIVED : {}".format(sample.payload))


STATE_LOST = 0
STATE_ALIGN_RIGHT = 1
STATE_ALIGN_LEFT = 2
STATE_FORWARD = 3
STATE_BACKWARD = 4
STATE_FINISH = 5


class MainView:
    def __init__(self, width, height, session):
        self.surface_configuration = (width, height)
        self.next_state = None
        self.session = session

        self.qcd = cv2.QRCodeDetector()

        self.camera_image_subscriber = self.session.declare_subscriber("turtle/camera", self.camera_image_callback)
        self.camera_image = None

        self.lidar_image_subscriber = self.session.declare_subscriber("turtle/lidar", self.lidar_scan_callback)
        self.lidar_image = None

        self.laser = Laser(360, 5, 359, 4000, 0, 0)
        self.map_size_meters = 5
        self.slam = RMHC_SLAM(self.laser, 600, self.map_size_meters)
        self.map = bytearray(600 * 600)
        self.pos = (0, 0, 0)

        self.cmd_vel_publisher = self.session.declare_publisher("turtle/cmd_vel")
        self.message_publisher = self.session.declare_publisher("turtle/debug_message")
        self.message_subscriber = self.session.declare_subscriber("turtle/debug_message", message_callback)

        self.interface = Interface()
        self.interface.add_gui(Used(pygame.K_UP, "↑", (200, 500), self.turtle_up, self.turtle_standby_up))
        self.interface.add_gui(Used(pygame.K_DOWN, "↓", (200, 550), self.turtle_down, self.turtle_standby_down))
        self.interface.add_gui(Used(pygame.K_LEFT, "←", (175, 525), self.turtle_left, self.turtle_standby_left))
        self.interface.add_gui(Used(pygame.K_RIGHT, "→", (225, 525), self.turtle_right, self.turtle_standby_right))

        self.last_distance = 0

        self.lidar_image = None

        self.last_points = []
        self.state = STATE_FINISH
        self.last_state = -1

    def quit(self):
        self.camera_image_subscriber.undeclare()
        self.lidar_image_subscriber.undeclare()
        self.cmd_vel_publisher.undeclare()
        self.message_publisher.undeclare()
        self.message_subscriber.undeclare()

    def camera_image_callback(self, sample):
        image = np.frombuffer(bytes(sample.value.payload), dtype=np.uint8)
        image = cv2.imdecode(image, 1)
        image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)

        ret_qr, decoded_info, points, _ = self.qcd.detectAndDecodeMulti(image)
        quad = points[0] if points is not None else None
        if points is not None:
            image = cv2.polylines(image, points.astype(int), True, (255, 0, 0), 3)

            distance = self.calcDistance(quad)

            self.last_distance = distance

        self.update_state(image.shape, quad)

        self.camera_image = pygame.surfarray.make_surface(image)

    def calcDistance(self, points):
        knownWidth = 100
        knownDistance = 40

        distanceBtw = lambda x, y: np.sqrt(np.dot(x - y, x - y))

        width = np.mean([distanceBtw(points[i], points[i + 1]) for i in range(3)])

        distance = (knownDistance * knownWidth) / width

        return distance

    def lidar_scan_callback(self, sample):
        scan = LaserScan.deserialize(sample.payload)

        angles = list(range(0, 360))

        # to mm
        distances = list(map(lambda x: x * 1000.0, scan.ranges))

        self.slam.update(scans_mm=distances, scan_angles_degrees=angles)
        self.slam.getmap(self.map)

        # transform into meters + translate in order to center the map
        self.pos = self.slam.getpos()
        self.pos = (self.pos[0] / 10, self.pos[1] / 10, self.pos[2])
        self.pos = (self.pos[0] - self.map_size_meters * 100 / 2, self.pos[1] - self.map_size_meters * 100 / 2, self.pos[2])

    def turtle_up(self):
        self.cmd_vel_publisher.put(("Forward", 10.0))

    def turtle_down(self):
        self.cmd_vel_publisher.put(("Forward", -10.0))

    def turtle_left(self):
        self.cmd_vel_publisher.put(("Rotate", 35.0))

    def turtle_right(self):
        self.cmd_vel_publisher.put(("Rotate", -35.0))

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

        if self.state != self.last_state:

            self.turtle_standby_down()
            self.turtle_standby_left()

            match self.state:
                case 0:
                    self.turtle_standby_up()
                case 1:
                    self.turtle_left()
                case 2:
                    self.turtle_right()
                case 3:
                    self.turtle_up()
                case 4:
                    self.turtle_down()
                case _:
                    pass

            self.last_state = self.state

    def update_state(self, imageShape, quad):
        ALIGNMENT_TOLERANCE = 75
        POSITION_TOLERANCE = 5

        width, height = imageShape[:2]

        if quad is None:
            self.state = STATE_LOST
            return

        position = np.mean(quad[:, 1])
        print(position)
        distance = self.calcDistance(quad)

        if position > width / 2 + ALIGNMENT_TOLERANCE:
            self.state = STATE_ALIGN_RIGHT
        elif position < width / 2 - ALIGNMENT_TOLERANCE:
            self.state = STATE_ALIGN_LEFT
        elif distance > 30 + POSITION_TOLERANCE:
            self.state = STATE_FORWARD
        elif distance < 30 - POSITION_TOLERANCE:
            self.state = STATE_BACKWARD
        else:
            self.state = STATE_FINISH

    def render(self, surface):
        surface.fill(IVORY)

        if self.camera_image is not None:
            surface.draw_rect(DARKBLUE, pygame.Rect(10, 10, self.camera_image.get_width() + 10,
                                                    self.camera_image.get_height() + 10))
            surface.blit(self.camera_image, 15, 15)

        if self.lidar_image is not None:
            surface.draw_rect(DARKBLUE, pygame.Rect(600, 20, self.lidar_image.get_width() + 10,
                                                    self.lidar_image.get_height() + 10))
            surface.blit(self.lidar_image, 605, 25)

        text = render_font(MOTO_MANGUCODE_30, f'Distance: {self.last_distance:.2f}cm', (0, 0, 0))

        surface.draw_image(text, 50, 400)

        self.interface.render(surface)
