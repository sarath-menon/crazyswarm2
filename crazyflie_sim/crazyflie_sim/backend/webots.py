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

import numpy as np
import rowan

from ament_index_python.packages import get_package_share_directory

import tf_transformations

class Backend:
    def __init__(self, node: Node, names: list[str], states: list[State]):
        self.node = node
        self.names = names
        self.clock_publisher = node.create_publisher(Clock, 'clock', 10)
        package_dir = get_package_share_directory('crazyflie_sim')

        #print(package_dir + "/worlds/crazyflie_world.wbt")
        #subprocess.Popen(["webots", package_dir + "/worlds/crazyflie_world.wbt"])

        os.environ['WEBOTS_CONTROLLER_URL'] = 'supervisor'

        self.supervisor = Supervisor()
        self.dt = int(self.supervisor.getBasicTimeStep())
        self.t = self.supervisor.getTime()

        # Get receiver and emitter
        self.receiver = self.supervisor.getDevice('receiver')
        self.receiver.enable(self.dt)
        self.emitter = self.supervisor.getDevice('emitter')

        root_node = self.supervisor.getRoot()
        children_field = root_node.getField('children')

        # Start webots
        h = 0
        self.cf_nodes = []
        for name in names:
            state = states[h]
            location = state.pos
            string_robot = ( 'DEF ' + name + ' Crazyflie {  translation '+str(location[0])+' ' + 
                str(location[1])+' '+str(location[2])+' name "'+ name  +'"  controller "<extern>"' + 
                'extensionSlot [ Receiver { } Emitter { } ]' + '}')
            children_field.importMFNodeFromString(-1, string_robot)
            args = [name]
            h = h + 1
            subprocess.Popen(["python3", package_dir + "/backend/webots_cf_driver.py"] + args)
            self.cf_nodes.append(self.supervisor.getFromDef(name))

    def time(self) -> float:
        return self.t

    def step(self, states_desired: list[State], actions: list[Action]) -> list[State]:
        #print(str(self.supervisor.getTime()) + "supervisor step")

        # Go through list of actions
        h = 0
        for action, name in zip(actions, self.names):
            #create a string with delimiters between the values
            ## rpm to rps
            rpss = [action.rpm[0]/60,action.rpm[1]/60,action.rpm[2]/60,action.rpm[3]/60]
            # multiply numpy array with a scalar float
            rpss = np.multiply(rpss, 1/7.0)
            #print(rpss)

            message = '['+name+'] ' + str(rpss[0]) + ' ' + str(rpss[1]) + ' ' + str(rpss[2]) + ' ' + str(rpss[3])
            self.emitter.send(message)

        # Step in the simulation, and get the next states
        self.supervisor.step(self.dt)

        '''imu_values = []
        while self.receiver.getQueueLength() > 0:
            message= self.receiver.getString()
            
            # Get string between brackets in message
            receipient_id = message[message.find('[')+1:message.find(']')]
            # Get the rest of the string
            message = message[message.find(']')+1:]
            # parse the values in a numpy array of floats
            values = np.fromstring(message, dtype=float, sep=' ')

            # save value with name in dictionary
            imu_values.append({receipient_id: values})
            
            self.receiver.nextPacket()'''

    

        next_states = []
        for name, node in zip(self.names,self.cf_nodes):
            next_state = State()
            next_state.pos = node.getPosition()
            velocity = node.getVelocity()
            next_state.vel = velocity[0:3]
            matrix = node.getOrientation()
            # find imu array in list of dictionaries
            '''if len(imu_values) > 0:
                imu_value = next((d[name] for d in imu_values if name in d), None)
            else:
                imu_value = [0,0,0]

            print('imu',imu_value)
            roll = imu_value[0]
            pitch = imu_value[1]
            yaw = imu_value[2]

            q_base = tf_transformations.quaternion_from_euler(roll,pitch, yaw)
            next_state.quat = q_base'''

            # make list into 3 x 3 matrix in numpy
            matrix = np.reshape(matrix, (3,3))

            #q_base= tf_transformations.quaternion_from_matrix(matrix)

            #print('rotation',tf_transformations.random_rotation_matrix())
            #next_state.quat = q_base

            next_state.quat = rowan.from_matrix(matrix)
            #print('quat',next_state.quat)



            # get angular velocity in world axis
            angular_velocity = velocity[3:6]
            # rotate angular velocity to body axis with rotation matrix
            angular_velocity_body = np.matmul(np.linalg.inv(matrix), angular_velocity)

            next_state.omega = angular_velocity_body

            next_states.append(next_state)

        self.t = self.supervisor.getTime()
        # publish the current clock
        clock_message = Clock()
        clock_message.clock = Time(seconds=self.t).to_msg()
        self.clock_publisher.publish(clock_message)

        
        return next_states



    def rotationMatrixToQuaternion1(self, m):
        #q0 = qw
        t = np.matrix.trace(m)
        q = np.asarray([0.0, 0.0, 0.0, 0.0], dtype=np.float64)

        if(t > 0):
            t = np.sqrt(t + 1)
            q[3] = 0.5 * t
            t = 0.5/t
            q[0] = (m[2,1] - m[1,2]) * t
            q[1] = (m[0,2] - m[2,0]) * t
            q[2] = (m[1,0] - m[0,1]) * t

        else:
            i = 0
            if (m[1,1] > m[0,0]):
                i = 1
            if (m[2,2] > m[i,i]):
                i = 2
            j = (i+1)%3
            k = (j+1)%3

            t = np.sqrt(m[i,i] - m[j,j] - m[k,k] + 1)
            q[i] = 0.5 * t
            t = 0.5 / t
            q[3] = (m[k,j] - m[j,k]) * t
            q[j] = (m[j,i] + m[i,j]) * t
            q[k] = (m[k,i] + m[i,k]) * t

        return q


    def shutdown(self):
        pass
