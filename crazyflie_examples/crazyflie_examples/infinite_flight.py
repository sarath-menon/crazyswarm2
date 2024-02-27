#!/usr/bin/env python

from pathlib import Path

from crazyflie_py import Crazyswarm
from crazyflie_py.uav_trajectory import Trajectory
import numpy as np
import time


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    # time.sleep(2)
    # for cf in allcfs.crazyflies:
    #     status =  cf.get_status()
    #     print(status)



        # from time import sleep
        # while True:
        #     status, counter = cf.get_status()
        #     print(status)
        #     sleep(0.1)
        # id, sec, nsec, s, b, p, r, rxb, txb, rxu, txu = status.values()
        # print(f"{cf.prefix} : {id}, {sec}, {nsec}, {s}, {b}, {p}, {r}, {rxb}, {rxu}, {txb}, {txu} ")

    traj1 = Trajectory()
    traj1.loadcsv(Path(__file__).parent / 'data/figure8.csv')

    TIMESCALE = 1.0
    for cf in allcfs.crazyflies:
        cf.uploadTrajectory(0, 0, traj1)

    allcfs.takeoff(targetHeight=1.0, duration=2.0)
    timeHelper.sleep(2.5)
    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([0, 0, 1.0])
        cf.goTo(pos, 0, 2.0)
    timeHelper.sleep(2.5)

    allcfs.startTrajectory(0, timescale=TIMESCALE)
    timeHelper.sleep(traj1.duration * TIMESCALE + 2.0)


    timeHelper.sleep(2.5)
    status = allcfs.status()        #NB what happens if we have multiple CFs ? 
    print(f"battery {status.battery}")

    ###second figure8
    allcfs.startTrajectory(0, TIMESCALE)
    timeHelper.sleep(traj1.duration * TIMESCALE + 2.0)


    allcfs.land(targetHeight=0.06, duration=2.0)
    timeHelper.sleep(3)
    status = allcfs.status()
    while status.pm_state != 1:
        print("Not charging, retrying")
        allcfs.takeoff(targetHeight=0.5, duration=1.0)
        timeHelper.sleep(1.5)
        allcfs.land(targetHeight=0.06, duration=1.0)
        timeHelper.sleep(2)

    print("charging !")
    timeHelper.sleep(3.0)


if __name__ == '__main__':
    main()
