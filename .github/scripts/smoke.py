#!/usr/bin/env python3
# Copyright 2026 Juan S. Cely G.
# Licensed under the Apache License, Version 2.0 (see LICENSE)
"""Smoke test of a launched gaitos robot through its ROS interface (CI).

    smoke.py <namespace> [--min-height H] [--min-distance D] [--vx V]

Waits for the odometry, stands up (mode/stand_up, then mode/balance_stand), walks with cmd_vel
and checks that the robot is standing, in `locomotion`, and moved forward. Times are simulated
(the stamps of the odometry), so a slow runner only makes it take longer.
"""
import argparse
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger

WALL_TIMEOUT = 240.0  # s, for each wait


class Smoke(Node):

    def __init__(self, ns):
        super().__init__('gaitos_smoke')
        self.ns = ns
        self.odom = None
        self.mode = ''
        self.lock = threading.Lock()
        self.create_subscription(Odometry, f'/{ns}/odometry', self._on_odom, 10)
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, f'/{ns}/mode', self._on_mode, latched)
        self.cmd = self.create_publisher(Twist, f'/{ns}/cmd_vel', 10)

    def _on_odom(self, msg):
        with self.lock:
            self.odom = msg

    def _on_mode(self, msg):
        self.mode = msg.data

    def sim_time(self):
        with self.lock:
            if self.odom is None:
                return None
            s = self.odom.header.stamp
            return s.sec + 1e-9 * s.nanosec

    def pose(self):
        with self.lock:
            p = self.odom.pose.pose.position
            return p.x, p.y, p.z

    def wait_until(self, what, cond):
        start = time.monotonic()
        while not cond():
            if time.monotonic() - start > WALL_TIMEOUT:
                raise RuntimeError(f'timeout waiting for {what}')
            time.sleep(0.05)

    def wait_sim(self, seconds, twist=None):
        t0 = self.sim_time()
        start = time.monotonic()
        while self.sim_time() - t0 < seconds:
            if time.monotonic() - start > WALL_TIMEOUT:
                raise RuntimeError('timeout: the simulation clock does not advance')
            if twist is not None:
                self.cmd.publish(twist)
            time.sleep(0.1)

    def call_mode(self, name):
        client = self.create_client(Trigger, f'/{self.ns}/mode/{name}')
        if not client.wait_for_service(timeout_sec=WALL_TIMEOUT):
            raise RuntimeError(f'service /{self.ns}/mode/{name} not available')
        res = client.call(Trigger.Request())
        print(f'mode/{name}: success={res.success} {res.message}', flush=True)
        if not res.success:
            raise RuntimeError(f'mode/{name} refused: {res.message}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('namespace')
    ap.add_argument('--min-height', type=float, default=0.22)
    ap.add_argument('--min-distance', type=float, default=0.3)
    ap.add_argument('--vx', type=float, default=0.3)
    args = ap.parse_args()

    rclpy.init()
    node = Smoke(args.namespace)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    ok = False
    try:
        node.wait_until('odometry', lambda: node.sim_time() is not None)
        print(f'odometry received, mode={node.mode!r}', flush=True)
        node.wait_sim(1.0)
        node.call_mode('stand_up')
        node.wait_sim(3.0)
        node.call_mode('balance_stand')
        node.wait_sim(1.0)
        x0, y0, z0 = node.pose()
        print(f'standing: z={z0:.3f} m, mode={node.mode}', flush=True)

        walk = Twist()
        walk.linear.x = args.vx
        node.wait_sim(5.0, walk)
        x1, y1, z1 = node.pose()
        mode = node.mode
        node.wait_sim(2.0, Twist())
        dist = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
        print(f'walked: {dist:.3f} m, z={z1:.3f} m, mode={mode}', flush=True)

        checks = {
            f'standing (z {z0:.3f} >= {args.min_height})': z0 >= args.min_height,
            f'still standing after walking (z {z1:.3f} >= {args.min_height})': z1 >= args.min_height,
            f'walking mode (locomotion, got {mode!r})': mode.startswith('locomotion'),
            f'moved forward ({dist:.3f} >= {args.min_distance} m)': dist >= args.min_distance,
        }
        for name, passed in checks.items():
            print(('PASS ' if passed else 'FAIL ') + name, flush=True)
        ok = all(checks.values())
        node.call_mode('damp')
    except RuntimeError as e:
        print(f'FAIL {e}', flush=True)
    finally:
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
