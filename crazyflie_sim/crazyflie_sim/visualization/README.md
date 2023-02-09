# Visualizations

## Blender

- Crazyflies to appear in scene are defined in `crazyswarm2/crazyflie/config/crazyflies.yaml` 
- Takes pictures from every CF's perspective and from a fixed camera observer in position $p = (0,0,1)$ with orientation $q = (1,0,0,0)$
- Saves pictures in working directory in `simulation_results/<date-and-time>/Raw-Dataset` with the following folder structure

    ```
    Raw-Dataset
    ├── cam
    │   ├── calibration.yaml
    │   ├── cam.csv
    │   ├── cam_00000.jpg
    │   ├── cam_00001.jpg
    │   └── ...
    ├── cf0
    │   ├── calibration.yaml
    │   ├── cf231.csv
    │   ├── cf231_00000.jpg
    │   ├── cf231_00001.jpg
    │   └── ...
    ├── ...
    └── cfn
        ├── calibration.yaml
        ├── cf1.csv
        ├── cf1_00000.jpg
        ├── cf1_00001.jpg
        └── ...
    ```
    where `<name>.csv` contains the states in world coordinates of the camera or crazyflie, `calibration.yaml` contains the calibration information of the cameras and 
    `<name>_<frame>.jpg` is the `<frame>`th image taken from `<names>`'s perspective. (Currently this corresponds to one picture every 400 time steps, but this will be
    customizable in the future.)
- In order to use it, you need to add `blender` to the list of visualizations in `crazyswarm2/crazyflie/config/server.yaml` and run 

    ```sh
    ros2 launch crazyflie launch.py backend:=sim
    ```






