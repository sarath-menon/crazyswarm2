from __future__ import annotations

import rclpy
import time
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from rclpy.time import Time
from ..sim_data_types import State, Action
from std_srvs.srv import Empty

import threading

webots_time = 0.0
lock = threading.Lock()

sys.path.insert(1, os.path.dirname(vehicle.__file__))
sys.path.insert(1, os.path.dirname(controller.__file__))
from controller import Supervisor  # noqa


class Backend:

    def __init__(self, node: Node, names: list[str], states: list[State]):
        self.node = node
        self.names = names
        self.clock_publisher = node.create_publisher(Clock, 'clock', 10)
        self.locked = True
        self.t = 0
        self.robot = Supervisor()
        self.dt = self.robot.getBasicTimeStep()

        self.uavs = []
        for state in states:
            uav = Quadrotor(state)
            self.uavs.append(uav)


    def sync_callback(self, request, response):
        print("hello")
        self.locked = False   
        return response     

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
        print(webots_time)
        clock_message.clock = Time(seconds=webots_time).to_msg()
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
        

