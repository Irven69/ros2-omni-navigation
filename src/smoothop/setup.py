from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'smoothop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'map'), glob('map/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motionControl = smoothop.motioncontrol:main',
            'motorControl = smoothop.motorcontrol:main',
            'odom = smoothop.odom:main',
            'test = smoothop.test:main',
            'motor_plotter = smoothop.motor_plotter:main',
            'pid_tuner = smoothop.pid_tuner:main',
            'joy_turbo_filter = smoothop.joy_turbo_filter:main',
        ],
    },
)
