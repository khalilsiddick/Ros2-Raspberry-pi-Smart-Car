from setuptools import setup # type: ignore
import os
from glob import glob

package_name = 'smart_car_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # Model files installation fix
        (os.path.join('share', package_name, 'models'), 
         glob('models/*.*')),  # Simplified model directory
        
        # Remove nested OpenCV_SSD_MobileNet structure - not needed
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='khalil',
    maintainer_email='alkhalilsiddick26@gmail.com',
    description='Example Python package for ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = smart_car_pkg.talker:main',
            'listener = smart_car_pkg.listener:main',
            'cmd_to_pwm_driver = smart_car_pkg.cmd_to_pwm_driver:main',
            'qr_control = smart_car_pkg.qr_control:main',
            'detection_node = smart_car_pkg.detection_node:main',
            'remote_control_node = smart_car_pkg.remote_control_node:main',
            'avoid_obstacles_node = smart_car_pkg.avoid_obstacles_node:main',
        ],
    },
)