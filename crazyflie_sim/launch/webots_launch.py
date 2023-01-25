import os
import pathlib
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.urdf_spawner import URDFSpawner, get_webots_driver_node
import yaml

def generate_launch_description():
    package_dir = get_package_share_directory('crazyflie_sim')
    supervisor_description = pathlib.Path(os.path.join(package_dir, 'resource', 'supervisor_webots.urdf')).read_text()

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'crazyflie_world.wbt')
    )

    supervisor = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_CONTROLLER_URL': 'Ros2Supervisor'},
        respawn=True,
        parameters=[
            {'robot_description': supervisor_description},
        ]
    )
    ld = LaunchDescription()
    ld.add_action(webots)
    ld.add_action(supervisor)



    return ld
