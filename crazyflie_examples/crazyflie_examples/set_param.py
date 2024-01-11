#!/usr/bin/env python

from crazyflie_py import Crazyswarm


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    # disable LED (one by one)
    for cf in allcfs.crazyflies:
        cf.setParam('led.bitmask', 128)
        timeHelper.sleep(1.0)

    timeHelper.sleep(2.0)

    # enable LED (broadcast)
    allcfs.setParam('led.bitmask', 0)
    timeHelper.sleep(5.0)


def set_param_allcfs(param, value):
    '''Sets a parameter for all crazyflies connected'''
    print("1")
    swarm = Crazyswarm()
    print("2")
    timeHelper = swarm.timeHelper
    print("3")
    allcfs = swarm.allcfs
    print("4")

    allcfs.setParam('led.bitmask', 128)
    allcfs.setParam(param, value)
    timeHelper.sleep(1.0)
    allcfs.setParam('led.bitmask', 0)

    print(f"Parameter {param} set to {value} on all crazyflies")

if __name__ == '__main__':
    # main()
    print("hi")
    set_param_allcfs("usd.logging", 1)
