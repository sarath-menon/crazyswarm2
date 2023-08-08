#!/usr/bin/env python

import numpy as np
from pathlib import Path

from crazyflie_py import *
from crazyflie_py.uav_trajectory import Trajectory

LOGGING = True

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    trajectories = []

    traj1 = Trajectory()
    traj1.loadcsv(Path(__file__).parent / "data/figur8.csv")
    trajectories.append(traj1)

    traj2 = Trajectory()
    traj2.loadcsv(Path(__file__).parent / "data/translation_x.csv")
    trajectories.append(traj2)

    traj3 = Trajectory()
    traj3.loadcsv(Path(__file__).parent / "data/translation_y.csv")
    trajectories.append(traj3)
    
    traj4 = Trajectory()
    traj4.loadcsv(Path(__file__).parent / "data/yaw0.csv")
    trajectories.append(traj4)

    TRAJ = 1
    TRIALS = 1
    TIMESCALE = 1.0
    for i in range(TRIALS):
        for cf in allcfs.crazyflies:
            cf.uploadTrajectory(0, 0, trajectories[TRAJ])

        allcfs.takeoff(targetHeight=1.0, duration=2.0)
        timeHelper.sleep(2.5)

        for cf in allcfs.crazyflies:
            pos = np.array(cf.initialPosition) + np.array([0, 0, 1.0])
            cf.goTo(pos, 0, 2.0)
        timeHelper.sleep(2.5)

        if LOGGING:
            print('Logging..')
            allcfs.setParam("usd.logging", 1)

        allcfs.startTrajectory(0, timescale=TIMESCALE)
        timeHelper.sleep(trajectories[TRAJ].duration * TIMESCALE + 2.0)
        # allcfs.startTrajectory(0, timescale=TIMESCALE, reverse=True)
        # timeHelper.sleep(traj1.duration * TIMESCALE + 2.0)

        if LOGGING:
            print("Logging done...")
            allcfs.setParam("usd.logging", 0)
            timeHelper.sleep(1)

        allcfs.land(targetHeight=0.06, duration=2.0)
        timeHelper.sleep(2.5)


if __name__ == "__main__":
    main()
