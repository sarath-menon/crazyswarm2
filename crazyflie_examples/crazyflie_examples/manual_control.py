#!/usr/bin/env python

from crazyflie_py import Crazyswarm
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist



class MinimalPublisher(Node):

    def __init__(self, swarm, allcfs, timeHelper):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_legacy', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.swarm = swarm
        self.allcfs = allcfs
        self.timeHelper = timeHelper

    def timer_callback(self):
        msg = Twist()

        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 40000.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        # cf = self.allcfs.crazyflies[0]
        # vel = np.ones(3)
        # cf.cmdVelocityWorld(vel, yawRate=0)


        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % self.i)
        self.i += 1

        # self.swarm._cfs["all"].cf.commander.send_setpoint(0,0,0,40000)

        if self.i==50:
            self.get_logger().info('Landing')
            self.allcfs.land(targetHeight=0.02, duration=1.0)
            self.timeHelper.sleep(1.0)
            


def main(args=None):
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    TAKEOFF_HEIGHT = 1.0
    TAKEOFF_DURATION = 2.0

    allcfs.takeoff(targetHeight=TAKEOFF_HEIGHT , duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION)

    minimal_publisher = MinimalPublisher(swarm, allcfs, timeHelper)

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
