from __future__ import annotations

import rclpy
import time
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from rclpy.time import Time
from ..sim_data_types import State, Action


import numpy as np
import rowan

class Backend:
    def __init__(self, node: Node, names: list[str], states: list[State]):
        self.node = node
        self.names = names
        self.clock_publisher = node.create_publisher(Clock, 'clock', 10)
        self.t = 0
        self.dt = 0.0005

        self.uavs = []
        for state in states:
            uav = Quadrotor(state)
            self.uavs.append(uav)

    def time(self) -> float:
        return self.t

    def step(self, states_desired: list[State], actions: list[Action]) -> list[State]:
        # advance the time
        self.t += self.dt

        next_states = []

        for uav, action in zip(self.uavs, actions):
            uav.step(action, self.dt)
            next_states.append(uav.state)

        # publish the current clock
        clock_message = Clock()
        # Replace clock here with webots clock
        self.clock_publisher.publish(clock_message)

        return next_states

    def shutdown(self):
        pass


class Quadrotor:
    """Basic rigid body quadrotor model (no drag) using numpy and rowan"""

    def __init__(self, state):
        print("hello! I'm a crazyflie!")

    def step(self, action, dt):
        print("step")
        

class WebotsSupervisor:
    def init(self, webots_node, properties):
        rclpy.init(args=None)
        self.node = rclpy.create_node('webots_supervisor')
        self.node.get_logger().info('Hello! Im the webots supervisor')

    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)
        self.node.get_logger().info('step')
        time.sleep(3)
