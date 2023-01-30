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
import sys

os.environ['WEBOTS_HOME'] = '/usr/local/webots'
os.environ['PYTHONPATH'] = os.path.expandvars('${WEBOTS_HOME}/lib/controller/python:$PYTHONPATH')
os.environ['PYTHONIOENCODING'] = 'UTF-8'

sys.path.append('/usr/local/webots/lib/controller/python')

from controller import Supervisor, Robot  # noqa
import threading


class Backend:
    def __init__(self, node: Node, names: list[str], states: list[State]):
        self.node = node
        self.names = names
        self.clock_publisher = node.create_publisher(Clock, 'clock', 10)
        self.locked = True
        os.environ['WEBOTS_CONTROLLER_URL'] = 'supervisor'
        print(os.getenv('WEBOTS_CONTROLLER_URL'))

        self.supervisor = Supervisor()
        self.dt = int(self.supervisor.getBasicTimeStep())
        self.t = self.supervisor.getTime()

        self.uavs = []

        root_node = self.supervisor.getRoot()
        children_field = root_node.getField('children')
        h = 0
        for name in names:
            state = states[h]
            location = state.pos
            string_robot =  'DEF ' + name + ' Crazyflie {  translation '+str(location[0])+' '+ str(location[1])+' '+str(location[2])+' name "'+ name +'"  controller "<extern>"}'
            children_field.importMFNodeFromString(-1, string_robot)
            thread = threading.Thread(target=self.create_quadcopter, args=( state, name))
            thread.start()
            h+=1

    def create_quadcopter(self, state, name):
        uav = Quadrotor(state, name)


    def time(self) -> float:
        return self.t

    def step(self, states_desired: list[State], actions: list[Action]) -> list[State]:

        self.supervisor.step(self.dt)
        self.t = self.supervisor.getTime()

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

    def __init__(self, state, name):
        print("hello! I'm a crazyflie!")
        self.state = state

        os.environ['WEBOTS_CONTROLLER_URL'] =  name
        print(os.getenv('WEBOTS_CONTROLLER_URL'))
        self.robot = Robot()


    def step_external(self):
        action = Action()
        dt = 0.1
        self.step(action,dt)

    def step(self, action, dt):
        print("step drone")
        

