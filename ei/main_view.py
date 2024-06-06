import cv2
import numpy as np
import pygame.image

from dataclasses import dataclass

from gfs.gui.interface import Interface
from gfs.gui.used import Used
from gfs.fonts import MOTO_MANGUCODE_10
from gfs.gui.button import *

from pycdr2 import IdlStruct
from pycdr2.types import uint32, float32
from typing import List

from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import Laser

import time


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


def calculate_distance_from_qr_code(points):
    known_width = 100
    known_distance = 40

    distance_btw = lambda x, y: np.sqrt(np.dot(x - y, x - y))

    width = np.mean([distance_btw(points[i], points[i + 1]) for i in range(3)])

    distance = (known_distance * known_width) / width

    return distance


# manual mode
MANUAL_MODE = 0

# qr code mode
QR_CODE_MODE = 1

STATE_LOST = 0
STATE_ALIGN_RIGHT = 1
STATE_ALIGN_LEFT = 2
STATE_FORWARD = 3
STATE_BACKWARD = 4
STATE_FINISH = 5

# go to position
LIDAR_MODE = 2


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
        self.map_image = None

        self.lidar_text = render_font(MOTO_MANGUCODE_10, "Instant Lidar Data", (0, 0, 0))
        self.map_text = render_font(MOTO_MANGUCODE_10, "Slam Map Data", (0, 0, 0))

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

        self.interface.add_gui(Button("Manual Mode", (400, 490), self.switch_to_manual))
        self.interface.add_gui(Button("QRcode Mode", (400, 540), self.switch_to_qrcode))
        self.interface.add_gui(Button("Lidar Mode", (400, 590), self.switch_to_lidar))

        self.PID_const = 0.6
        self.qr_code_center_x = 0
        self.distance_to_qr_code = 0

        self.last_points = []
        self.state = STATE_FINISH
        self.last_state = -1

        self.destination = np.array([0, 0])
        self.position = np.zeros(2)
        self.angle = 0
        self.last_angle = 0
        self.cumulative_error_distance = 0
        self.cumulative_error_angle = 0

        self.mode = MANUAL_MODE

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
        image = cv2.flip(image, 0)

        ret_qr, decoded_info, points, _ = self.qcd.detectAndDecodeMulti(image)
        quad = points[0] if points is not None else None

        if points is not None:
            image = cv2.polylines(image, points.astype(int), True, (255, 0, 0), 3)

            self.distance_to_qr_code = calculate_distance_from_qr_code(quad)
            self.qr_code_center_x = np.mean(quad[:, 0])

        self.update_state(image.shape, quad)

        self.camera_image = pygame.surfarray.make_surface(image)

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
        self.pos = (
            self.pos[0] - self.map_size_meters * 100 / 2, self.pos[1] - self.map_size_meters * 100 / 2, self.pos[2])

        # transform map bytearray into a pygame image
        map_image = np.array(self.map).reshape((600, 600))
        _, map_image = cv2.threshold(map_image, 100, 255, cv2.THRESH_BINARY)
        map_image = cv2.rotate(map_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        map_image = cv2.cvtColor(map_image, cv2.COLOR_GRAY2RGB)

        x = int(300 + self.pos[1])
        y = int(300 - self.pos[0])

        map_image = cv2.circle(map_image, (x, y), 10, (0, 0, 255), -1)

        map_image = cv2.resize(map_image, (300, 300))

        self.map_image = pygame.surfarray.make_surface(map_image)

        # draw instant scan on a pygame image

        lidar_image = np.zeros((600, 600, 3), dtype=np.uint8)

        for i, distance in enumerate(distances):
            if distance < 750:
                # fit the distance inside the window
                real_distance = distance / 750.0 * 300.0

                angle = np.radians(angles[i])
                x = int(300.0 + real_distance * np.cos(angle))
                y = int(300.0 + real_distance * np.sin(angle))

                lidar_image = cv2.circle(lidar_image, (x, y), 10, (0, 255, 0), -1)

        lidar_image = cv2.circle(lidar_image, (300, 300), 10, (255, 255, 255), -1)
        lidar_image = cv2.rotate(lidar_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        lidar_image = cv2.resize(lidar_image, (300, 300))
        self.lidar_image = pygame.surfarray.make_surface(lidar_image)

    def update_state(self, image_shape, quad):
        alignment_tolerance = 50
        position_tolerance = 5

        width, height = image_shape[:2]

        if quad is None:
            self.state = STATE_LOST
            return

        position = np.mean(quad[:, 1])
        distance = calculate_distance_from_qr_code(quad)

        if position > width / 2 + alignment_tolerance:
            self.state = STATE_ALIGN_RIGHT
        elif position < width / 2 - alignment_tolerance:
            self.state = STATE_ALIGN_LEFT
        elif distance > 30 + position_tolerance:
            self.state = STATE_FORWARD
        elif distance < 30 - position_tolerance:
            self.state = STATE_BACKWARD
        else:
            self.state = STATE_FINISH

    def set_destination(self, dest):
        self.destination = dest

    def set_movement(self, linear, angular):
        self.cmd_vel_publisher.put(("Forward", linear))
        self.cmd_vel_publisher.put(("Rotate", angular))

    def go_to_destination(self):
        alignment_tolerance = 4  # degree
        position_tolerance = 5  # cm

        if self.destination is None:
            self.set_movement(0.0, 0.0)
            return

        position = self.pos[0], self.pos[1]
        angle = self.pos[2]

        x, y = self.destination - position
        relative_position_angle = (np.pi + np.arctan(y / x) if x >= 0 else np.arctan(y / x)) * 180 / np.pi
        relative_angle = relative_position_angle - angle

        relative_distance = np.sqrt(x ** 2 + y ** 2)

        if abs(relative_angle) > alignment_tolerance and relative_distance > position_tolerance:
            alpha = 1 * relative_angle + 2 * (relative_angle - self.last_angle) + .04 * self.cumulative_error_angle
            self.set_movement(0.0, float(alpha))

            self.cumulative_error_angle += (relative_angle - self.last_angle)
            self.last_angle = relative_angle
        elif relative_distance > position_tolerance:
            self.set_movement(20.0, 0.0)

        else:
            self.set_movement(0.0, 0.0)

    def turtle_up(self):
        if self.mode == MANUAL_MODE:
            self.cmd_vel_publisher.put(("Forward", 20.0))

    def turtle_down(self):
        if self.mode == MANUAL_MODE:
            self.cmd_vel_publisher.put(("Forward", -20.0))

    def turtle_left(self):
        if self.mode == MANUAL_MODE:
            self.cmd_vel_publisher.put(("Rotate", 100.0))

    def turtle_right(self):
        if self.mode == MANUAL_MODE:
            self.cmd_vel_publisher.put(("Rotate", -100.0))

    def turtle_standby_up(self):
        if self.mode == MANUAL_MODE:
            self.cmd_vel_publisher.put(("Forward", 0.0))

    def turtle_standby_down(self):
        if self.mode == MANUAL_MODE:
            self.cmd_vel_publisher.put(("Forward", 0.0))

    def turtle_standby_left(self):
        if self.mode == MANUAL_MODE:
            self.cmd_vel_publisher.put(("Rotate", 0.0))

    def turtle_standby_right(self):
        if self.mode == MANUAL_MODE:
            self.cmd_vel_publisher.put(("Rotate", 0.0))

    def switch_to_manual(self):
        self.mode = MANUAL_MODE

    def switch_to_qrcode(self):
        self.mode = QR_CODE_MODE

    def switch_to_lidar(self):
        self.mode = LIDAR_MODE

    def keyboard_input(self, event):
        self.interface.keyboard_input(event)

    def mouse_input(self, event):
        self.interface.mouse_input(event)

        if event.button == 1:
            pos = np.array(event.pos) - np.array((965, 405))
            pos = (pos / 300) * self.map_size_meters - self.map_size_meters / 2
            pos = pos * 100
            pos[0] = -pos[0]

            if -self.map_size_meters * 100 / 2 < pos[0] < self.map_size_meters * 100 / 2:
                if -self.map_size_meters * 100 / 2 < pos[1] < self.map_size_meters * 100 / 2:
                    self.destination = pos
                    print(f"Click at: {pos}")

    def mouse_motion(self, event):
        self.interface.mouse_motion(event)

    def update(self):
        self.interface.update()

        if self.mode == QR_CODE_MODE:
            if self.state != self.last_state:

                self.cmd_vel_publisher.put(("Forward", 0.0))
                self.cmd_vel_publisher.put(("Rotate", 0.0))

                vel_x = self.PID_const * abs(self.qr_code_center_x - self.camera_image.get_width() / 2)

                match self.state:

                    case 1:
                        self.cmd_vel_publisher.put(("Rotate", -vel_x))
                    case 2:
                        self.cmd_vel_publisher.put(("Rotate", vel_x))
                    case 3:
                        self.cmd_vel_publisher.put(("Forward", 20.0))
                    case 4:
                        self.cmd_vel_publisher.put(("Forward", -20.0))
                    case _:
                        pass

                self.last_state = self.state

        elif self.mode == LIDAR_MODE:
            self.go_to_destination()

    def render(self, surface):
        surface.fill(IVORY)

        if self.camera_image is not None:
            surface.draw_rect(DARKBLUE, pygame.Rect(10, 10, self.camera_image.get_width() + 10,
                                                    self.camera_image.get_height() + 10))
            surface.blit(self.camera_image, 15, 15)

        if self.lidar_image is not None:
            surface.draw_rect(DARKBLUE, pygame.Rect(960, 20, self.lidar_image.get_width() + 10,
                                                    self.lidar_image.get_height() + 10))
            surface.blit(self.lidar_image, 965, 25)

            surface.draw_image(self.lidar_text, 1050, 335)

        if self.map_image is not None:
            surface.draw_rect(DARKBLUE, pygame.Rect(960, 400, self.map_image.get_width() + 10,
                                                    self.map_image.get_height() + 10))
            surface.blit(self.map_image, 965, 405)

            surface.draw_image(self.map_text, 1075, 385)

        text = render_font(MOTO_MANGUCODE_30, f'Distance: {self.distance_to_qr_code:.2f}cm', (0, 0, 0))

        surface.draw_image(text, 50, 400)

        self.interface.render(surface)
