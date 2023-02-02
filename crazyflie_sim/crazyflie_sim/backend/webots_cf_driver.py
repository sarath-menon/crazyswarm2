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

import numpy as np

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
        self.m2_motor = self.robot.getDevice("m2_motor")
        self.m2_motor.setPosition(float('inf'))
        self.m2_motor.setVelocity(1)
        self.m3_motor = self.robot.getDevice("m3_motor")
        self.m3_motor.setPosition(float('inf'))
        self.m3_motor.setVelocity(-1)
        self.m4_motor = self.robot.getDevice("m4_motor")
        self.m4_motor.setPosition(float('inf'))
        self.m4_motor.setVelocity(1)

        self.receiver = self.robot.getDevice("receiver")
        self.receiver.enable(self.time_step)
        self.emitter = self.robot.getDevice("emitter")

        self.imu = self.robot.getDevice("inertial unit")
        self.imu.enable(self.time_step)
    
    def set_motor_velocity(self, motor, velocity):
        if abs(velocity) > 600:
            velocity = 600 * np.sign(velocity)
        
        motor.setVelocity(velocity)


    def step(self):

        
        while self.receiver.getQueueLength() > 0:
            message= self.receiver.getString()
            
            # Get string between brackets in message
            receipient_id = message[message.find('[')+1:message.find(']')]
            if receipient_id == self.name or receipient_id == 'all':
                # Get the rest of the string
                message = message[message.find(']')+1:]
                # parse the values in a numpy array of floats
                values = np.fromstring(message, dtype=float, sep=' ')
                #print (values)
                #print(self.name + ' received: ' + message + '\n')

                self.set_motor_velocity(self.m1_motor, -1 * values[0])
                self.set_motor_velocity(self.m2_motor, values[1])
                self.set_motor_velocity(self.m3_motor,-1* values[2])
                self.set_motor_velocity(self.m4_motor, values[3])

            
            self.receiver.nextPacket()
        
        # Get IMU data
        #imu_data = self.imu.getRollPitchYaw()

        # Send IMU data
        #message = '['+name+'] ' + str(imu_data[0]) + ' ' + str(imu_data[1]) + ' ' + str(imu_data[2])
        #self.emitter.send(message)
            
    
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
