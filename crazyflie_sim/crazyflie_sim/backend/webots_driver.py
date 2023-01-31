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

os.environ['WEBOTS_HOME'] = '/usr/local/webots'
os.environ['PYTHONPATH'] = os.path.expandvars('${WEBOTS_HOME}/lib/controller/python:$PYTHONPATH')
os.environ['PYTHONIOENCODING'] = 'UTF-8'

sys.path.append('/usr/local/webots/lib/controller/python')

from controller import Supervisor, Robot  # noqa

class CrazyflieWebotsDriver:
    def __init__(self, name):
        os.environ['WEBOTS_CONTROLLER_URL'] = name
        self.name = name
        self.robot = Robot()
        self.time_step = int(self.robot.getBasicTimeStep())
        self.m1_motor = self.robot.getDevice("m1_motor")
        self.m1_motor.setPosition(float('inf'))
        self.m1_motor.setVelocity(-1)
        self.receiver = self.robot.getDevice("receiver")
        self.receiver.enable(self.time_step)
        self.emitter = self.robot.getDevice("emitter")


    def step(self):
        if name == 'cf5':
            self.m1_motor.setVelocity(1)
        
        while self.receiver.getQueueLength() > 0:
            message= self.receiver.getString()
            
            # Get string between brackets in message
            receipient_id = message[message.find('[')+1:message.find(']')]
            if receipient_id == self.name or receipient_id == 'all':
                #print(self.name + ' received: ' + message + '\n')

                return_message = str(self.robot.getTime()) + '[sup] ' + self.name + ' received: ' + message
                self.emitter.send(return_message)
            
            self.receiver.nextPacket()


if __name__ == '__main__':

    name = sys.argv[1]
    webots_driver = CrazyflieWebotsDriver(name)
    time_step = int(webots_driver.robot.getBasicTimeStep())
    webots_driver.step()

    # Keep looping until the simulation is over
    # This step goes before the supervisor step
    while webots_driver.robot.step(time_step) != -1:
        #print(str(webots_driver.robot.getTime())+ webots_driver.robot.name +' step')
        webots_driver.step()
        pass
