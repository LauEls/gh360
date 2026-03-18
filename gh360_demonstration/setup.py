from setuptools import setup
import os
from glob import glob

package_name = 'gh360_demonstration'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
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
            'step_pub = gh360_demonstration.step_pub:main',
            'step_pub_2 = gh360_demonstration.step_pub_2:main',
            'timer_test = gh360_demonstration.timer_test:main',
            'gui = gh360_demonstration.gui:main',
            'door_env_obs = gh360_demonstration.door_env_obs:main',
            'robosuite_demo = gh360_demonstration.robosuite_demo:main',
            'gh360_demo = gh360_demonstration.gh360_demo:main',
            'gh360_sim_demo = gh360_demonstration.gh360_sim_demo:main',
            'spacemouse = gh360_demonstration.spacemouse:main',
            'gh360_erf_teleop = gh360_demonstration.gh360_erf_teleop:main',
        ],
    },
)
