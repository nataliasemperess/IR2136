from setuptools import setup

package_name = 'ros_drone_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Natalia',
    author_email='al415537@uji.es',
    description='Control de dron con ROS2 y MAVLink',
    license='MIT',
    entry_points={
        'console_scripts': [
            'battery_gps_node = ros_drone_control.battery_gps_node:main',
            'mission_control_node = ros_drone_control.mission_control_node:main',
        ],
    },
)

