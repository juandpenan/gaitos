#!/usr/bin/env python3
# Copyright 2026 Juan S. Cely G.
# Licensed under the Apache License, Version 2.0 (see LICENSE)
"""Save the images of a sensor_msgs/Image topic as numbered PNG files (documentation clips).

    save_images.py <topic> <folder>
"""
import os
import sys

import rclpy
from PIL import Image as PilImage
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def main():
    topic, folder = sys.argv[1], sys.argv[2]
    os.makedirs(folder, exist_ok=True)
    rclpy.init()
    n = Node('doc_image_saver')
    count = [0]

    def save(m):
        if m.encoding not in ('rgb8', 'bgr8'):
            return
        im = PilImage.frombytes('RGB', (m.width, m.height), bytes(m.data))
        if m.encoding == 'bgr8':
            b, g, r = im.split()
            im = PilImage.merge('RGB', (r, g, b))
        im.save(os.path.join(folder, f'frame_{count[0]:05d}.png'))
        count[0] += 1
    n.create_subscription(Image, topic, save, qos_profile_sensor_data)
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
