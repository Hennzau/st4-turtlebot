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

        self.lidar_image_subscriber = self.session.declare_subscriber("turtle/lidar", self.lidar_image_callback)
        self.lidar_image = None

        self.cmd_vel_publisher = self.session.declare_publisher("turtle/cmd_vel")
        self.message_publisher = self.session.declare_publisher("turtle/debug_message")
        self.message_subscriber = self.session.declare_subscriber("turtle/debug_message", message_callback)

        self.interface = Interface()
        self.interface.add_gui(Used(pygame.K_UP, "↑", (200, 500), self.turtle_up, self.turtle_standby_up))
        self.interface.add_gui(Used(pygame.K_DOWN, "↓", (200, 550), self.turtle_down, self.turtle_standby_down))
        self.interface.add_gui(Used(pygame.K_LEFT, "←", (175, 525), self.turtle_left, self.turtle_standby_left))
        self.interface.add_gui(Used(pygame.K_RIGHT, "→", (225, 525), self.turtle_right, self.turtle_standby_right))

        self.last_distance = 0

        fig, ax = plt.subplots()
        self.figure = fig
        self.ax = ax

        self.patch = ax.add_patch(Polygon([[0, 0]], color='lightgrey'))
        self.line = ax.plot([], [], '.', color='black')[0]
        center = ax.plot([0], [0], 'o', color='blue')[0]
        ax.set_xlim(-4, 4)
        ax.set_ylim(-4, 4)

        canvas = FigureCanvas(self.figure)
        canvas.draw()
        renderer = canvas.get_renderer()
        matplotlib_plot_rgba_image_data = renderer.tostring_rgb()  # Get raw image data of the plot
        plot_canvas_width, plot_canvas_height = canvas.get_width_height()

        self.lidar_image = pygame.image.fromstring(matplotlib_plot_rgba_image_data,
                                                   (plot_canvas_width, plot_canvas_height),
                                                   "RGB")
        
        self.last_points = []
        self.state = STATE_FINISH
        self.last_state = -1
        
        self.destination = np.array([50,30])
        self.position = np.zeros(2)
        self.angle = 0
        self.last_distance = 0
        self.last_angle = 0
        self.cumilative_error_distance = 0
        self.cumilative_error_angle = 0

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

            distance = self.calc_distance(quad)

            self.last_distance = distance
            
        self.update_state(image.shape, quad)
            
        
        self.camera_image = pygame.surfarray.make_surface(image)

    def set_destination(self, dest):
        self.destination = dest
        
    def set_mouvement(self, linear, angular):
        self.cmd_vel_publisher.put(("Forward", linear))
        self.cmd_vel_publisher.put(("Rotate", angular))
    
    def go_to_destination(self):
        ALIGNMENT_TOLERANCE = 4 #degree
        POSITION_TOLERANCE  = 5 #cm
        
        if self.position is None:
            return
        
        relative_position = self.destination - self.position
        relative_angle = self.angle - np.arctan2(*relative_position)
        relative_distance = np.sqrt(np.dot(relative_position,relative_position))
        
        if abs(relative_angle) > ALIGNMENT_TOLERANCE:
            
            alpha = 10*relative_angle + 4*self.cumilative_error_angle +4*(relative_angle - self.last_angle)
            self.set_mouvement(0,alpha)
            
            self.last_angle = relative_angle
            self.cumilative_error_angle += (relative_angle - self.last_angle)
        elif relative_distance > POSITION_TOLERANCE:
            v = 100*relative_position + 10*self.cumilative_error_angle +10*(relative_angle - self.last_angle)
            self.set_mouvement(0,v)
            
            self.last_distance = relative_distance
            self.cumilative_error_distance += (relative_distance- self.last_distance)
        else:
            self.set_mouvement(0, 0)
            
    
    def calc_distance(self, points):
        knownWidth = 100
        knownDistance = 40

        distanceBtw = lambda x,y: np.sqrt(np.dot(x-y,x-y))
        
        width = np.mean([distanceBtw(points[i],points[i+1]) for i in range(3)])
        
        distance = (knownDistance * knownWidth) / width

        return distance

    def lidar_image_callback(self, sample):
        print('[DEBUG] Received frame: {}'.format(sample.key_expr))

        scan = LaserScan.deserialize(sample.payload)
        angles = list(
            map(lambda x: x * 1j + cmath.pi / 2j, np.arange(scan.angle_min, scan.angle_max, scan.angle_increment)))

        complexes = []
        for (angle, distance, intensity) in list(zip(angles, scan.ranges, scan.intensities)):
            complexes.append(distance * cmath.exp(angle) if intensity >= 250.0 else 1024 * cmath.exp(angle))
        X = [i.real for i in complexes]
        Y = [i.imag for i in complexes]
        XY = [[i.real, i.imag] for i in complexes]

        self.patch.set_xy(XY)
        self.line.set_data(X, Y)

        canvas = FigureCanvas(self.figure)
        canvas.draw()
        renderer = canvas.get_renderer()
        matplotlib_plot_rgba_image_data = renderer.tostring_rgb()  # Get raw image data of the plot
        plot_canvas_width, plot_canvas_height = canvas.get_width_height()

        self.lidar_image = pygame.image.fromstring(matplotlib_plot_rgba_image_data,
                                                   (plot_canvas_width, plot_canvas_height),
                                                   "RGB")

        """
                self.lidar_image =
                # Convert Matplotlib plot to Pygame surface
                buffer = io.BytesIO()
                self.fig.savefig(buffer, format="png")
                buffer.seek(0)
                image = Image.open(buffer)
                mode = image.mode
                size = image.size
                data = image.tobytes()
                self.lidar_image = pygame.image.fromstring(data, size, mode)
        """

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
                    self.turtle_right()
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
        
        position = np.mean(quad[:,1])
        print(position)
        distance = self.calc_distance(quad)
        
        if position > width / 2 + ALIGNMENT_TOLERANCE:
            self.state = STATE_ALIGN_RIGHT
        elif position < width / 2 - ALIGNMENT_TOLERANCE:
            self.state = STATE_ALIGN_LEFT
        elif distance > 30 + POSITION_TOLERANCE :
            self.state = STATE_FORWARD
        elif distance < 30 - POSITION_TOLERANCE :
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
            surface.draw_rect(DARKBLUE, pygame.Rect(600, 130, self.lidar_image.get_width() + 10,
                                                    self.lidar_image.get_height() + 10))
            surface.blit(self.lidar_image, 605, 135)

        text = render_font(MOTO_MANGUCODE_30, f'Distance: {self.last_distance:.2f}cm', (0, 0, 0))

        surface.draw_image(text, 50, 400)

        self.interface.render(surface)
