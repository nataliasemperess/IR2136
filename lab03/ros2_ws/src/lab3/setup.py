from setuptools import setup

package_name = 'lab3'

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
    maintainer='natalia',
    maintainer_email='al415537@uji.es',
    description='LAB3',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'mission_control = lab3.mission_control_node:main',
             'battery_gps = lab3.battery_gps_node:main',
        ],
    },
)
