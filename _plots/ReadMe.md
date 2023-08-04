# How to create reports with custom plots?

## 1. settings.yaml

- be sure to have the folders ```/reports``` and ```/logs``` in the directory ```_plots```
- adjust the following general settings
    - ```data_file```
    - ```event_name```
    - ```output_dir```
- figures can be defined in the figure settings
    - copy and change the following parameters
        - ```title```
        - ```type``` (only 2d and 3d for now)
        - ```x_label```
        - ...
        - each structure within a figure dictionary defines a subplot
            - the x-axis data of the subplot is defined by the key, i.e. ```timestamp``` which is the name of the time data wihtin the log file
            - each subplot can have multiple signals which can be defined by another dictionary with the key ```y_info```
                - it has the keys ```data``` and ```label```
                - ```data``` defines the name of the signalss within the log file by its keys, the legend names of the signals can be defined with the respective values
- title settings can also be adjusted
    - ```title_settings```: change the names of the settings which should be displayed in the title
    - ```title_params```: define the parameters which should be displayed in the title
- change the unit settings
    - ```convert_units```: define the units which should be converted and how, the value represents the conversion factor
    - ```units```: define the string which should be displayed as the unit (not in use yet)

## 2. plot.py

```bash
cd _plots
python plot.py
```

It should output something similar to this:

```bash
...creating figures
start_time: 143230.548
output path: reports/log29.pdf
processing event: fixedFrequency (0)
>>> created figure 1: Positions
>>> created figure 2: Translation Velocities
>>> created figure 3: Angles
>>> created figure 4: Angular Velocities
>>> created figure 5: Thrust
>>> created figure 6: Torques (xyz)
>>> created figure 7: Trajectories
...done creating figures
```