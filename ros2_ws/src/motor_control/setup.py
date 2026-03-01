from glob import glob
from setuptools import find_packages, setup

package_name = 'motor_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='David Soni',
    maintainer_email='sonidavid46@gmail.com',
    description='Micro-ROS motor control package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'motor_commander = motor_control.motor_commander:main',
            'motor_monitor = motor_control.motor_monitor:main'
        ],
    },
)
