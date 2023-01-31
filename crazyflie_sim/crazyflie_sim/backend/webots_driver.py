from __future__ import annotations

import rclpy
import time
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from rclpy.time import Time
from std_srvs.srv import Empty
import os
import sys

webots_time = 0.0


import os
import sys

#os.environ['WEBOTS_HOME'] = '/usr/local/webots'
#os.environ['PYTHONPATH'] = os.path.expandvars('${WEBOTS_HOME}/lib/controller/python:$PYTHONPATH')
#os.environ['PYTHONIOENCODING'] = 'UTF-8'

sys.path.append('/usr/local/webots/lib/controller/python')

from controller import Supervisor, Robot  # noqa


        
class CrazyflieWebotsDriver:
    def __init__(self, name):
        os.environ['WEBOTS_CONTROLLER_URL'] = 'ipc://1234/cf231'
        self.robot = Robot()

    def step(self):
        print('crazyflie step')


if __name__ == '__main__':
    webots_driver = CrazyflieWebotsDriver('cf321')
    while True:
        webots_driver.step()