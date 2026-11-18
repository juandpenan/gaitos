#!/usr/bin/env python3
# Copyright 2026 Juan S. Cely G.
# Licensed under the Apache License, Version 2.0 (see LICENSE)
"""Drive a gaitos robot through its ROS interface with a list of steps (documentation clips).

    drive.py <namespace> <step>...
    steps: mode:<name> | vel:<vx>,<vy>,<wz> (cmd_vel at 10 Hz until the next vel/stop) | stop
           | wait:<seconds> | pose:<z>,<roll>,<pitch> (goal_pose in the base frame)
"""
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_srvs.srv import Trigger


def main():
    ns, steps = sys.argv[1], sys.argv[2:]
    rclpy.init()
    n = Node('doc_driver')
    cmd = n.create_publisher(Twist, f'/{ns}/cmd_vel', 10)
    goal = n.create_publisher(PoseStamped, f'/{ns}/goal_pose', 10)
    twist = {'msg': None}
    threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()

    def pump():
        while rclpy.ok():
            if twist['msg'] is not None:
                cmd.publish(twist['msg'])
            time.sleep(0.1)
    threading.Thread(target=pump, daemon=True).start()
    for step in steps:
        kind, _, arg = step.partition(':')
        if kind == 'mode':
            c = n.create_client(Trigger, f'/{ns}/mode/{arg}')
            c.wait_for_service(timeout_sec=10.0)
            print(step, c.call(Trigger.Request()).success, flush=True)
        elif kind == 'vel':
            vx, vy, wz = (float(x) for x in arg.split(','))
            t = Twist()
            t.linear.x, t.linear.y, t.angular.z = vx, vy, wz
            twist['msg'] = t
        elif kind == 'stop':
            twist['msg'] = Twist()
        elif kind == 'wait':
            time.sleep(float(arg))
        elif kind == 'pose':
            z, roll, pitch = (float(x) for x in arg.split(','))
            g = PoseStamped()
            g.header.frame_id = ''
            g.pose.position.z = z
            g.pose.orientation.x = roll / 2
            g.pose.orientation.y = pitch / 2
            g.pose.orientation.w = 1.0
            goal.publish(g)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
