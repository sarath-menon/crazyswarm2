#!/usr/bin/env python

from crazyflie_py import Crazyswarm
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from crazyflie_online_tracker_interfaces.msg import CommandOuter, ControllerState, CrazyflieState
import signal

class IfaSim(Node):

    def __init__(self, swarm):
        super().__init__('minimal_publisher')

        # publishers and subscribers   
        self.setpoint_sub = self.create_subscription(CommandOuter, "controllerCommand", self.callback_command, 10)
        self.publisher_ = self.create_publisher(Twist, '/cf231/cmd_vel_legacy', 10)

        self.i = 0

        self.swarm = swarm
        self.allcfs = swarm.allcfs

        signal.signal(signal.SIGINT, self.exit_handler)

        TAKEOFF_HEIGHT = 0.4
        TAKEOFF_DURATION = 2.0

        # # takeoff
        # self.allcfs.takeoff(targetHeight=TAKEOFF_HEIGHT , duration=TAKEOFF_DURATION)
        # self.swarm.timeHelper.sleep(2.0)

        timeHelper = swarm.timeHelper
        self.swarm.timeHelper.sleep(TAKEOFF_DURATION)
    
    def exit_handler(self, signum, frame):
        print("Landing crazyflie and exiting program")
        self.allcfs.land(targetHeight=0.00, duration=2.0)
        self.swarm.timeHelper.sleep(2.0)
        exit()

    def callback_command(self, msg: CommandOuter):

        if msg.is_last_command == True:
            print("Received land command")

            self.allcfs.land(targetHeight=0.00, duration=2.0)
            self.swarm.timeHelper.sleep(2.0)
            

        elif msg.is_takeoff == True:
            print("Received takeoff commands")
            self.allcfs.takeoff(targetHeight=0.4, duration=2.0)
            self.swarm.timeHelper.sleep(2.0)

        else:
            msg_out = Twist()
            msg_out.linear.z = msg.thrust # thrust (0-65000)
            msg_out.angular.x = msg.omega.x #roll 
            msg_out.angular.y = msg.omega.y #pitch 
            msg_out.angular.z = msg.omega.z #yaw 

            self.publisher_.publish(msg_out)
            self.i += 1

def main(args=None):
    swarm = Crazyswarm()

    minimal_publisher = IfaSim(swarm)

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

