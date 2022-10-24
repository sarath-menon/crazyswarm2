"""Takeoff-hover-land for one CF. Useful to validate hardware config."""

# from pycrazyswarm import Crazyswarm
from py_crazyswarm2 import Crazyswarm
import numpy as np



Ids = [2, 5, 6]
# TAKEOFF_HEIGHT = 1.3490287
TAKEOFF_HEIGHT = 1.24045172
LAND_HEIGHT    = 0.5
Heights = [0.85778483, 0.85778483]

# offset UAVs from payload for 20 degrees
offsets = {
    2: {
        'offset' : [ 0, -0.33809461,  0.72504623], #offset for 25 degrees  
                                            
    },
    5: {
        'offset': [0.29279852, 0.1690473,  0.72504623],
    
    },
    6: {
        'offset' : [-0.29279852, 0.1690473,  0.72504623],
    }
}

cf_config = {
    2: {
        'waypoints': [
            [0, -0.33809461, 1.22504623],  
        ]                                  

    },
    5: {
        'waypoints': [
            [0.29279852, 0.1690473, 1.22504623], 
        ]                                  

    },
    6: {
        'waypoints': [
            [-0.29279852, 0.1690473, 1.22504623],                             
        ]                                      
    }
}

cf_values = {
    2: {
        'value' : 0,
    },
    5: {
        'value' : 1,
    },
    6: {
        'value' : 2,
    }
}

lengths = {
    2: {
        'length' : 0.8,
    },
    5: {
        'length' : 0.8,
    },
    6: {
        'length' : 0.8,
    }
}

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    for cfid in Ids:
        allcfs.crazyfliesById[cfid].takeoff(targetHeight=TAKEOFF_HEIGHT, duration=5.0)
    timeHelper.sleep(5.5)
    
    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([0, 0, TAKEOFF_HEIGHT])
        cf.goTo(pos, 0, 3.0)

    timeHelper.sleep(5.0)

    for cfid in Ids:
        print(cfid)
        pos = np.array(cf_config[cfid]['waypoints'][0])
        print(cfid, pos)
        allcfs.crazyfliesById[cfid].goTo(pos, 0, 5.0)
    timeHelper.sleep(7.0) 
    
   
#### CHANGE the values of the UAVs and the offset for the setpoints

  
    for cfid in Ids: 
        value = int(cf_values[cfid]['value'])
        offset = offsets[cfid]['offset']
        print('value and ID: ', value, cfid)
        allcfs.crazyfliesById[cfid].setParam('ctrlLeeP.value', value)
        allcfs.crazyfliesById[cfid].setParam('ctrlLeeP.offsetx', offset[0])
        allcfs.crazyfliesById[cfid].setParam('ctrlLeeP.offsety', offset[1])
        allcfs.crazyfliesById[cfid].setParam('ctrlLeeP.offsetz', offset[2])

    timeHelper.sleep(5.5)

    print('start hovering with QP lee payload')
    

    for cfid in Ids: 
        allcfs.crazyfliesById[cfid].setParam("usd.logging", 1)
        allcfs.crazyfliesById[cfid].setParam('stabilizer.controller', 7)

    
    timeHelper.sleep(1.0)

###### Linear trajectory on x,y -axes #######################
    # for cfid in Ids:
    #     pos = np.array(cf_config[cfid]['waypoints'][1])
    #     allcfs.crazyfliesById[cfid].goTo(pos, 0, 15.0)
    
    # timeHelper.sleep(17.5)

    # for cfid in Ids:
    #     pos = np.array(cf_config[cfid]['waypoints'][0])
    #     allcfs.crazyfliesById[cfid].goTo(pos, 0, 15.0)
    
    # timeHelper.sleep(17.5)
 ################################################################   
    print('get back to Lee')

    for cfid in Ids: 
        allcfs.crazyfliesById[cfid].setParam("usd.logging", 0)
        allcfs.crazyfliesById[cfid].setParam('stabilizer.controller', 6)


    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([0, 0, LAND_HEIGHT])
        cf.goTo(pos, 0, 3.0)
    timeHelper.sleep(2.0)


    allcfs.land(targetHeight=0.02, duration=3.0)
    timeHelper.sleep(3.0)



# TAKEOFF_DURATION = 10.0
# PAYLOAD_CONTROLLER = 30.0
# HOVER_BACK     = 5.0
# TARGET_HEIGHT   = 1.0

    # cf = swarm.allcfs.crazyflies[0]
    # cf.setParam('stabilizer.controller', 6)


    # cf.takeoff(targetHeight=TARGET_HEIGHT, duration=TAKEOFF_DURATION)
    # timeHelper.sleep(TAKEOFF_DURATION)    
    
    
    # cf.setParam("usd.logging", 1)
    # print('start hovering with lee payload')
    # cf.setParam('stabilizer.controller', 7)

    # timeHelper.sleep(PAYLOAD_CONTROLLER)
    
    # # cf.goTo([0.5,0.0,0.0], 0.0, 2.0, relative=True)
    # # timeHelper.sleep(3.0)
    
    # # cf.goTo([-0.5,0.0,0.0], 0.0, 2.0, relative=True)
    # # timeHelper.sleep(3.0)

    # print('finished trajectory')
    # cf.setParam("usd.logging", 0) 

    # print('swap controller')
    # cf.setParam('ctrlLee.mass', 0.034+0.007)
    # cf.setParam('stabilizer.controller', 6)

    # initPos = cf.initialPosition

    # cf.goTo(initPos+[0,0,TARGET_HEIGHT], 0, HOVER_BACK)

    # timeHelper.sleep(HOVER_BACK)
    
    # cf.land(targetHeight=0.04, duration=2.5)
    
    # timeHelper.sleep(TAKEOFF_DURATION)


if __name__ == "__main__":
    main()
