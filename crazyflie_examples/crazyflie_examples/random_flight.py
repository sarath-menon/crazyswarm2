#!/usr/bin/python3
import numpy as np
from crazyflie_py import *
import threading
import time
import shutil
import os
import subprocess 


def move(cf, last_pos, total_time, bbox_min, bbox_max):
    start = time.time()
    done = False
    while not done:
        if time.time() < start + total_time:
            pos = np.random.uniform(bbox_min, bbox_max)
        else:
            # move back to the initial position at the end pos = np.array(cf.initialPosition) + np.array([0, 0, 0.5])
            done = True

        dist = np.linalg.norm(last_pos - pos)
        speed = np.random.uniform(0.1, 0.5) # m/s
        time_to_move = max(dist / speed, 1.0)
        cf.goTo(pos, 0, time_to_move)
        last_pos = pos
        time.sleep(time_to_move)

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    # allcfs.crazyfliesById[7].emergency()
    # allcfs.crazyfliesById[2].emergency()
    # allcfs.crazyfliesById[231].emergency()
    # allcfs.crazyfliesById[3].emergency()


    #folder_name = 'test9'
    #path = '/home/whoenig/tmp/cvmrs/random_flight/' + folder_name + '/Raw-Dataset/'
    #shutil.rmtree(path, ignore_errors=True)
    #os.makedirs(path)
    run_file = "/home/whoenig/projects/tuberlin/cv-mrs/dataset-collection/real-data/data_collection_aideck.py"

    cf_config = {
        6: {
            #'IP': "130.149.82.49", #aideck-6
            # calibration not great
            #'calibration': "/home/whoenig/tubCloud/projects/cvmrs/camera-calibration/aideck6_1/cf6/calibration.yaml",
            # 'bbox_min': [0.0,0.0,0.3],
            # 'bbox_max': [0.0,0.0,0.3],
        },
        5: {
            #'IP':"130.149.82.50", # aideck-7
            #'calibration': "/home/whoenig/tubCloud/projects/cvmrs/camera-calibration/aideck7_2/cf7/calibration.yaml",
            # 'bbox_min': [0.0,0.0,0.4],
            # 'bbox_max': [0.0,0.0,0.4],
        },
        7: {
            # 'bbox_min': [-0.5,-0.5,0.2],
            # 'bbox_max': [0.5,0.5,0.6],
        },
        8: {
            # 'bbox_min': [-0.5,-0.5,0.2],
            # 'bbox_max': [0.5,0.5,0.6],
        },
        231: {
            # 'bbox_min': [-0.5,-0.5,0.2],
            # 'bbox_max': [0.5,0.5,0.6],
        }
    }
    cfids = list(allcfs.crazyfliesById.keys())
    IPs = [cf_config[cfid]['IP'] for cfid in cfids if 'IP' in cf_config[cfid]]
    IDs = [cfid for cfid in cfids if 'IP' in cf_config[cfid]]
    CALIB_FILES = [cf_config[cfid]['calibration']  for cfid in cfids if 'IP' in cf_config[cfid]] 
    folder_list = [os.path.join(path, "cf{}".format(cfid)) for cfid in cfids if 'IP' in cf_config[cfid]]
    IPs_FOLDERs_IDs = [(ip, folder, str(id)) for ip, folder, id in zip(IPs, folder_list, IDs)]
    bbox_min = np.array([-0.5,-0.5,0.2])
    bbox_max = np.array([0.5,0.5,1.25])

    allcfs.setParam("colAv.ellipsoidX", 0.15)
    allcfs.setParam("colAv.ellipsoidY", 0.15)
    allcfs.setParam("colAv.ellipsoidZ", 0.3)

    allcfs.setParam("colAv.bboxMinX", bbox_min[0])
    allcfs.setParam("colAv.bboxMinY", bbox_min[1])
    allcfs.setParam("colAv.bboxMinZ", bbox_min[2])

    allcfs.setParam("colAv.bboxMaxX", bbox_max[0])
    allcfs.setParam("colAv.bboxMaxY", bbox_max[1])
    allcfs.setParam("colAv.bboxMaxZ", bbox_max[2])

    # creating folders for each CF images separately
    #for _, folder_name, _ in IPs_FOLDERs_IDs:
    #    os.mkdir(os.path.join(path, folder_name)) 

    allcfs.setParam("usec.reset", 1)

    allcfs.setParam("cvmrs.trigger", 1)

    cmds_list = [["python3", run_file, "-n", ip, "-path", os.path.join(path, folder_name), "-id", id] for ip, folder_name, id in IPs_FOLDERs_IDs]
    procs_list = [subprocess.Popen(cmd) for cmd in cmds_list]
    
    for folder in folder_list:
        while True:
            time.sleep(1)
            if len(os.listdir(folder)) > 1:
                break  

    # start recording to sdcard
    allcfs.setParam("usd.logging", 1)
    timeHelper.sleep(2.)
    
    # for the flight part
    allcfs.takeoff(targetHeight=0.5, duration=1.0)
    timeHelper.sleep(1.0)

    # enable collision avoidance
    allcfs.setParam("colAv.enable", 1)

    # # # auto-yaw for aidecks ONLY
    # for cfid in IDs:
    #     allcfs.crazyfliesById[cfid].setParam("hlCommander.yawrate", 0.2)

    allcfs.setParam("hlCommander.yawrate", 2.0)


    # start thread for each cf
    threads = []
    total_time = 1200
    for cfid, cf in allcfs.crazyfliesById.items():
        cf_bbox_min = bbox_min
        cf_bbox_max = bbox_max
        if cfid in cf_config:
            cfg = cf_config[cfid]
            if 'bbox_min' in cfg:
                cf_bbox_min = np.asarray(cfg['bbox_min'])
            if 'bbox_max' in cfg:
                cf_bbox_max = np.asarray(cfg['bbox_max'])
        pos = np.array(cf.initialPosition) + np.array([0, 0, 0.5])
        thread = threading.Thread(target=move, args=(cf, pos, total_time, cf_bbox_min, cf_bbox_max))
        thread.start()
        threads.append(thread)

    # wait for all threads to be done
    for thread in threads:
        thread.join()

    # stop auto-yaw
    # for cfid in IDs:
    #     allcfs.crazyfliesById[cfid].setParam("hlCommander.yawrate", 0.0)

    allcfs.setParam("hlCommander.yawrate", 0.0)
    timeHelper.sleep(1.0)

    # go to initial positions (and set yaw to zero)
    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([0, 0, 0.5])
        cf.goTo(pos, 0, 1.0)
    timeHelper.sleep(1.0)

    # disable collision avoidance
    allcfs.setParam("colAv.enable", 0)
   
    allcfs.land(targetHeight=0.02, duration=1.0)
    timeHelper.sleep(1.0)

    # stop recording to sdcard
    allcfs.setParam("usd.logging", 0)
    timeHelper.sleep(2.0)

    for proc in procs_list:
        proc.terminate()

    # save calibration file
    for i in range(len(IDs)):
        shutil.copy(CALIB_FILES[i], os.path.join(path, folder_name) +'/calibration.yaml') 

if __name__ == "__main__":
    main()
