# import the opencv library
import time

import cv2
import imutils
import numpy as np
import zenoh

from imutils.video import VideoStream

config = zenoh.Config.from_file("config.json")
session = zenoh.open(config)

publisher = session.declare_publisher("turtle/camera")

vs = VideoStream(src=0).start()
jpeg_opts = [int(cv2.IMWRITE_JPEG_QUALITY), 95]

time.sleep(1.0)

while True:
    raw = vs.read()
    if raw is not None:
        frame = imutils.resize(raw, width=400)
        frame = np.rot90(frame)
        _, jpeg = cv2.imencode('.jpg', frame, jpeg_opts)

        publisher.put(jpeg.tobytes())

    time.sleep(0.016)
