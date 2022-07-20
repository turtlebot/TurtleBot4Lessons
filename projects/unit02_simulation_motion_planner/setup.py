from setuptools import setup
import os
from glob import glob

package_name = 'unit02_simulation_motion_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name,'worlds'), glob('worlds/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sumedh',
    maintainer_email='sumedhrk@umd.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
           'tb4_rider = unit02_simulation_motion_planner.tb4_planner:main',
           'ekf = unit02_simulation_motion_planner.extented_kalman_filter:main',
           'tb4_goal = unit02_simulation_motion_planner.tb4_publish_goal:main',
        ],
    },
)
