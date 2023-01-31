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
import subprocess

os.environ['WEBOTS_HOME'] = '/usr/local/webots'
os.environ['PYTHONPATH'] = os.path.expandvars('${WEBOTS_HOME}/lib/controller/python:$PYTHONPATH')
os.environ['PYTHONIOENCODING'] = 'UTF-8'

sys.path.append('/usr/local/webots/lib/controller/python')

from controller import Supervisor, Robot  # noqa
import threading

from ament_index_python.packages import get_package_share_directory

class Backend:
    def __init__(self, node: Node, names: list[str], states: list[State]):
        self.node = node
        self.names = names
        self.clock_publisher = node.create_publisher(Clock, 'clock', 10)
        self.locked = True

        os.environ['WEBOTS_CONTROLLER_URL'] = 'supervisor'

        self.supervisor = Supervisor()
        self.dt = int(self.supervisor.getBasicTimeStep())
        self.t = self.supervisor.getTime()

        self.uavs = []

        root_node = self.supervisor.getRoot()
        children_field = root_node.getField('children')
        package_dir = get_package_share_directory('crazyflie_sim')

        h = 0
        for name in names:
            state = states[h]
            location = state.pos
            string_robot = ( 'DEF ' + name + ' Crazyflie {  translation '+str(location[0])+' ' + 
                str(location[1])+' '+str(location[2])+' name "'+ name  +'"  controller "<extern>"' + 
                'extensionSlot [ Receiver { } Emitter { } ]' + '}')
            children_field.importMFNodeFromString(-1, string_robot)
            uav = Quadrotor(state, name)
            self.uavs.append(uav)
            args = [name]
            h = h + 1
            subprocess.Popen(["python3", package_dir + "/backend/webots_driver.py"] + args)

    def time(self) -> float:
        return self.t

    def step(self, states_desired: list[State], actions: list[Action]) -> list[State]:
        #print(str(self.supervisor.getTime()) + "supervisor step")
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

    def step(self, action, dt):
        print("step drone")
        