#!/usr/bin/env python

from crazyflie_py import Crazyswarm
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from crazyflie_online_tracker_interfaces.msg import CommandOuter, ControllerState, CrazyflieState
from crazyflie_interfaces.msg import FullState
import signal

class IfaSim(Node):

    def __init__(self, swarm):
        super().__init__('minimal_publisher')

        # publishers and subscribers   
        self.setpoint_sub = self.create_subscription(FullState, "controllerCommand", self.callback_command, 10)
        self.cmd_vel_publisher_ = self.create_publisher(FullState, '/cf231/cmd_vel_legacy', 10)

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

        # if msg.is_last_command == True:
        #     print("Received land command")

        #     self.allcfs.land(targetHeight=0.00, duration=2.0)
        #     self.swarm.timeHelper.sleep(2.0)
            

        # elif msg.is_takeoff == True:
        #     print("Received takeoff commands")
        #     self.allcfs.takeoff(targetHeight=0.4, duration=2.0)
        #     self.swarm.timeHelper.sleep(2.0)

        # else:
        msg_out = FullState()
        msg_out.header.stamp = msg.header.stamp
        msg_out.header.frame_id = "world"

        msg_out.pose.position.x = msg.pose.position.x
        msg_out.pose.position.y = msg.pose.position.y
        msg_out.pose.position.z = msg.pose.position.z

        msg_out.twist.linear.x = msg.twist.linear.x
        msg_out.twist.linear.y = msg.twist.linear.y
        msg_out.twist.linear.z = msg.twist.linear.z
        
        msg_out.twist.angular.x = msg.twist.angular.x
        msg_out.twist.angular.y = msg.twist.angular.y
        msg_out.twist.angular.z = msg.twist.angular.z

        msg_out.acc.x = 0.0
        msg_out.acc.y = 0.0
        msg_out.acc.z = msg.acc.z # thrust (0-65000)

        self.cmd_vel_publisher_.publish(msg_out)
        self.i += 1

    # def callback_command(self, msg: CommandOuter):

    #     if msg.is_last_command == True:
    #         print("Received land command")

    #         self.allcfs.land(targetHeight=0.00, duration=2.0)
    #         self.swarm.timeHelper.sleep(2.0)
            

    #     elif msg.is_takeoff == True:
    #         print("Received takeoff commands")
    #         self.allcfs.takeoff(targetHeight=0.4, duration=2.0)
    #         self.swarm.timeHelper.sleep(2.0)

    #     else:
    #         msg_out = Twist()
    #         msg_out.linear.z = msg.thrust # thrust (0-65000)
    #         msg_out.angular.x = msg.omega.x #roll 
    #         msg_out.angular.y = msg.omega.y #pitch 
    #         msg_out.angular.z = msg.omega.z #yaw 

    #         self.publisher_.publish(msg_out)
    #         self.i += 1

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

