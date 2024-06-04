import cv2
import numpy as np
import pygame.image
import io
from PIL import Image
import cmath

from gfs.gui.interface import Interface
from gfs.gui.used import Used
from gfs.pallet import IVORY, DARKBLUE

from dataclasses import dataclass
from pycdr2 import IdlStruct
from pycdr2.types import int8, int32, uint32, float32, float64
from typing import List


@dataclass
class Vector3(IdlStruct, typename="Vector3"):
    x: float64
    y: float64
    z: float64


@dataclass
class Twist(IdlStruct, typename="Twist"):
    linear: Vector3
    angular: Vector3
    
    
    
class Time(IdlStruct, typename="Time"):
    sec: uint32
    nsec: uint32
    
class Header(IdlStruct, typename="Header"):
    stamp: Time
    frame_id: str

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




def calculate_twist(linear, angular):
    t = Twist(linear=Vector3(x=linear, y=0.0, z=0.0),
              angular=Vector3(x=0.0, y=0.0, z=angular))

    return t.serialize()


def message_callback(sample):
    print("MESSAGE RECEIVED : {}".format(sample.payload))
    


class MainView:
    def __init__(self, width, height, session):
        self.surface_configuration = (width, height)
        self.next_state = None
        self.session = session

        self.camera_image_subscriber = self.session.declare_subscriber("turtle/camera", self.camera_image_callback)
        self.camera_image = None
        
        self.lidar_image_subscriber = self.session.declare_subscriber("turtle/lidar", self.lidar_image_callback)
        self.lidar_image = None

        self.cmd_vel_publisher = self.session.declare_publisher("turtle/cmd_vel")
        self.message_publisher = self.session.declare_publisher("turtle/debug_message")
        self.message_subscriber = self.session.declare_subscriber("turtle/debug_message", message_callback)

        self.interface = Interface()
        self.interface.add_gui(Used(pygame.K_UP, "↑", (200, 500), self.turtle_up))
        self.interface.add_gui(Used(pygame.K_DOWN, "↓", (200, 550), self.turtle_down))
        self.interface.add_gui(Used(pygame.K_LEFT, "←", (175, 525), self.turtle_left))
        self.interface.add_gui(Used(pygame.K_RIGHT, "→", (225, 525), self.turtle_right))

    def quit(self):
        self.camera_image_subscriber.undeclare()
        self.lidar_image_subscriber.undecalre()
        self.cmd_vel_publisher.undeclare()
        self.message_publisher.undeclare()
        self.message_subscriber.undeclare()

    def camera_image_callback(self, sample):
        image = np.frombuffer(bytes(sample.value.payload), dtype=np.uint8)
        image = cv2.imdecode(image, 1)
        image = np.rot90(image)

        self.camera_image = pygame.surfarray.make_surface(image)
    
    def lidar_image_callback(self, sample):
        print('[DEBUG] Received frame: {}'.format(sample.key_expr))
        scan = LaserScan.deserialize(sample.payload)
        angles = list(map(lambda x: x*1j+cmath.pi/2j, np.arange(scan.angle_min, scan.angle_max, scan.angle_increment)))

        complexes = []
        for (angle, distance, intensity) in list(zip(angles, scan.ranges, scan.intensities)):
            complexes.append(distance * cmath.exp(angle) if intensity >= 250.0 else 1024 * cmath.exp(angle))
        X = [i.real for i in complexes]
        Y = [i.imag for i in complexes]
        XY = [[i.real, i.imag] for i in complexes]
        self.patch.set_xy(XY)
        self.line.set_data(X, Y)

        # Convert Matplotlib plot to Pygame surface
        buffer = io.BytesIO()
        self.fig.savefig(buffer, format="png")
        buffer.seek(0)
        image = Image.open(buffer)
        mode = image.mode
        size = image.size
        data = image.tobytes()
        self.lidar_image = pygame.image.fromstring(data, size, mode)
    

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

        self.message_publisher.put("Debug message")

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
            
        if self.lidar_image is not None:
            surface.draw_rect(DARKBLUE, pygame.Rect(800, 10, self.lidar_image.get_width() + 10,
                                                    self.lidar_image.get_height() + 10))
            surface.blit(self.lidar_image, (805, 25))

        self.interface.render(surface)
