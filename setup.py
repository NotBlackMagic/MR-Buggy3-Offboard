import os
from glob import glob
from setuptools import setup

package_name = 'locomotion'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='NotBlackMagic',
    maintainer_email='fabian_naef@live.com',
    description='Locomotion controller for the MR-Buggy3. Gets positions and published RC/Manual commands to the PX4 autopilot.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
			'mavlink_bridge = locomotion.mavlink_bridge:main',
			'msg_trans = locomotion.message_translation:main',
            'odometry = locomotion.odometry:main',
            'vel_ctrl = locomotion.velocity_controller:main',
			'keyboard_ctrl = locomotion.keyboard_control:main',
        ],
    },
)
