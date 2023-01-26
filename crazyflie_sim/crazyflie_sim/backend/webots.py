from __future__ import annotations

import rclpy
import time
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from rclpy.time import Time
from ..sim_data_types import State, Action
from std_srvs.srv import Empty
import os
import sys

webots_time = 0.0


import os
os.environ['WEBOTS_HOME'] = '/usr/local/webots'
os.environ['PYTHONPATH'] = '${WEBOTS_HOME}/lib/controller/python38:$PYTHONPATH'
os.environ['PYTHONIOENCODING'] = 'UTF-8'

from controller import Supervisor  # noqa


class Backend:

    def __init__(self, node: Node, names: list[str], states: list[State]):
        self.node = node
        self.names = names
        self.clock_publisher = node.create_publisher(Clock, 'clock', 10)
        self.locked = True
        self.robot = Supervisor()
        self.dt = int(self.robot.getBasicTimeStep())
        self.t = self.robot.getTime()

        self.uavs = []
        for state in states:
            uav = Quadrotor(state)
            self.uavs.append(uav)

    def time(self) -> float:
        return self.t

    def step(self, states_desired: list[State], actions: list[Action]) -> list[State]:

        self.robot.step(self.dt)
        self.t = self.robot.getTime()

        next_states = []
        #for uav, action in zip(self.uavs, actions):
        #    uav.step(action, self.dt)
        #    next_states.append(uav.state)

        # publish the current clock
        clock_message = Clock()
        clock_message.clock = Time(seconds=self.t).to_msg()
        self.clock_publisher.publish(clock_message)

        return next_states

    def shutdown(self):
        pass


class Quadrotor:

    def __init__(self, state):
        print("hello! I'm a crazyflie!")
        self.state = state

    def step(self, action, dt):
        print("step drone")
        

