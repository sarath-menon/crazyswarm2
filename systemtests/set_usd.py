from crazyflie_py import Crazyswarm

def set_param(param, value):
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    allcfs.setParam('led.bitmask', 128)    

    timeHelper.sleep(2.0)
    allcfs.setParam(param, value)
    # enable LED (broadcast)
    allcfs.setParam('led.bitmask', 0)



if __name__ == "__main__":
    set_param("usd.logging", 1)
    import time
    time.sleep(2)
    set_param("usd.logging", 0)
