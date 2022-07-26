"""Takeoff-hover-land for one CF. Useful to validate hardware config."""

# from pycrazyswarm import Crazyswarm
from py_crazyswarm2 import Crazyswarm


TAKEOFF_DURATION = 10.0
PAYLOAD_CONTROLLER = 30.0
HOVER_BACK     = 5.0
TARGET_HEIGHT   = 1.0

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]
    cf.setParam('stabilizer.controller', 6)


    cf.takeoff(targetHeight=TARGET_HEIGHT, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION)    
    
    
    cf.setParam("usd.logging", 1)
    print('start hovering with lee payload')
    cf.setParam('stabilizer.controller', 7)

    timeHelper.sleep(PAYLOAD_CONTROLLER)
    
    # cf.goTo([0.5,0.0,0.0], 0.0, 2.0, relative=True)
    # timeHelper.sleep(3.0)
    
    # cf.goTo([-0.5,0.0,0.0], 0.0, 2.0, relative=True)
    # timeHelper.sleep(3.0)

    print('finished trajectory')
    cf.setParam("usd.logging", 0) 

    print('swap controller')
    cf.setParam('ctrlLee.mass', 0.034+0.007)
    cf.setParam('stabilizer.controller', 6)

    initPos = cf.initialPosition

    cf.goTo(initPos+[0,0,TARGET_HEIGHT], 0, HOVER_BACK)

    timeHelper.sleep(HOVER_BACK)
    
    cf.land(targetHeight=0.04, duration=2.5)
    
    timeHelper.sleep(TAKEOFF_DURATION)


if __name__ == "__main__":
    main()
