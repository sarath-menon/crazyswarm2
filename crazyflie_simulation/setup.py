from setuptools import setup

package_name = 'crazyflie_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Wolfgang Hönig, Kimberly McGuire',
    maintainer_email='hoenig@tu-berlin.de',
    description='The Crazyflie ROS2 simulation package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'crazyflie_webots_driver = crazyflie_simulation.crazyflie_webots_driver:main'
        ],
    },
)
