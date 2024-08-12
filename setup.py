from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'gesture_cam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='manan',
    maintainer_email='themananrughani@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'img_publisher = gesture_cam.webcam_pub:main',
            'img_subscriber = gesture_cam.webcam_sub:main',
            'gesture_detection = gesture_cam.gesture_detection:main',
            'gesture_control = gesture_cam.gesture_control:main'
        ],
    },
)
