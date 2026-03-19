from setuptools import setup, find_packages
import os
import sys
from glob import glob

package_name = 'gh360_examples'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='laurenz',
    maintainer_email='laurenz.elstner@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'monitor = gh360_examples.monitor:main',
            'door_reset = gh360_examples.door.reset:main',
            'door_handle_angle_filter = gh360_examples.door.handle_angle_filter:main',
            'door_handle_pose = gh360_examples.door.handle_pose:main',
            'door_env_obs = gh360_examples.door.env_obs:main',
            
            'tendon_model_generator = gh360_examples.tendon_model:main',
            'real_time_plot = gh360_examples.real_time_plot:main',
            'point_vis = gh360_examples.point_vis:main',
            'camera_vis = gh360_examples.camera_vis:main',
            'camera_frame = gh360_examples.camera_frame:main',
            'pos_step_pub = gh360_examples.pos_step_pub:main',
            'testing_script = gh360_examples.testing_script:main',
            'reset_robot = gh360_examples.reset_robot:main',
            'robosuite_teleop = gh360_examples.robosuite_teleop:main',
            'erf_leaderboard = gh360_examples.erf_leaderboard:main',
            'send_time = gh360_examples.send_time:main',
            'serial_encoder_handler = gh360_examples.serial_encoder_handler:main'
        ],
    },
)
