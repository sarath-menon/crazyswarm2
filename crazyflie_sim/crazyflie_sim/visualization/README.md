# Visualizations

## Blender

### Dependencies

- `bpy`
- `scipy` 
- `rowan`
- `numpy` 
- `yaml` 

### Output
- Takes pictures from robots' perspectives and (optionally) from a fixed camera observer
- Saves pictures in working directory in `simulation_results/<date-and-time>/Raw-Dataset` with the following folder structure

    ```
    Raw-Dataset
    ├── cam
    │   ├── calibration.yaml
    │   ├── cam.csv
    │   ├── cam_00000.jpg
    │   ├── cam_00001.jpg
    │   └── ...
    ├── cf0
    │   ├── calibration.yaml
    │   ├── cf231.csv
    │   ├── cf231_00000.jpg
    │   ├── cf231_00001.jpg
    │   └── ...
    ├── ...
    └── cfn
        ├── calibration.yaml
        ├── cf1.csv
        ├── cf1_00000.jpg
        ├── cf1_00001.jpg
        └── ...
    ```
    where `<name>.csv` contains the states in world coordinates of the camera or crazyflie, `calibration.yaml` contains the calibration information of the cameras and 
    `<name>_<frame>.jpg` is the `<frame>`th image taken from `<names>`'s perspective. If a robot is configured to not carry a camera, only `<name>.csv` will be recorded. 
- In order to use it, you need to add `blender` to the list of visualizations in `crazyswarm2/crazyflie/config/server.yaml` and run 

    ```sh
    ros2 launch crazyflie launch.py backend:=sim
    ```

### Configuration
- Crazyflies to appear in scene are defined in `crazyflie/config/crazyflies.yaml` 
- In `crazyflie/config/server.yaml` you **must** set the following parameters, unless stated otherwise:
    * `enabled: boolean`, enables or disables blender visualization in simulator
    * `fps: float`, frames per second rate of all cameras  
    * `auto_yaw: boolean`, enables or disables auto-yaw (optional, defaults to `false` if not set)
        - `radps: float`, radians per second
    * for every robot in `cf_cameras`:
        - `calibration`
            * `camera_matrix: list[float]`, camera matrix as list in row-major order
            * `dist_coeff: list[float]` 
            * `tvec: list[float]` 
            * `rvec: list[float]` 
- Example configuration  
    ```yaml
    blender:
      enabled: true
      auto_yaw:     # constant yaw rotation applied to cf_cameras only
        enabled: true
        radps: 5   # radians per second
      fps: 1           # frames per second
      cf_cameras:      # names of crazyflies with cameras on them if enabled in `crazyflies.yaml`
        cf231:
          calibration:
            camera_matrix: [170.0, 0.0, 160.0, 0.0, 170.0, 160.0, 0.0, 0.0, 1.0] # matrix in row-major order
            dist_coeff: [0,0,0,0,0]
            tvec: [0,0,0]
            rvec: [1.2092,-1.2092,1.2092]   # 0 deg tilt
        cf5:
          calibration:
            camera_matrix: [170.0, 0.0, 160.0, 0.0, 170.0, 160.0, 0.0, 0.0, 1.0] # matrix in row-major order
            dist_coeff: [0,0,0,0,0]
            tvec: [0,0,0]
            rvec: [ 0.61394313, -0.61394313,  1.48218982]   # 45 deg tilt
        cf6:
          calibration:
            camera_matrix: [170.0, 0.0, 160.0, 0.0, 170.0, 160.0, 0.0, 0.0, 1.0] # matrix in row-major order
            dist_coeff: [0,0,0,0,0]
            tvec: [0,0,0]
            rvec: [0.0,0.0,1.5707963267948966]    # 90 deg tilt
      observer:        # static observer camera 
        enabled: false
        pos: [-0.1,0,1]
        quat: [1,0,0,0]
        calibration:
          camera_matrix: [170.0, 0.0, 160.0, 0.0, 170.0, 160.0, 0.0, 0.0, 1.0] # matrix in row-major order
          dist_coeff: [0,0,0,0,0]
          tvec: [0,0,0]
          rvec: [ 0.61394313, -0.61394313, 1.48218982]
    ```
