#!/usr/bin/env python
from crazyflie_py import Crazyswarm
from argparse import ArgumentParser, Namespace

def set_param_allcfs(param, value):
    '''Sets a parameter for all connected crazyflies'''
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    #turn off LEDs
    allcfs.setParam('led.bitmask', 128)
    #set parameter
    allcfs.setParam(param, value)
    timeHelper.sleep(1.0)
    #turn on LEDs
    allcfs.setParam('led.bitmask', 0)
    print(f"Parameter {param} set to {value} on all crazyflies")

if __name__ == "__main__":
    parser = ArgumentParser(description="Set a parameter for all connected crazyflies")
    parser.add_argument("param", type=str, help="Name of the parameter to set")
    parser.add_argument("value", help="Value to set the parameter to")
    args : Namespace = parser.parse_args()
    set_param_allcfs(args.param, args.value)