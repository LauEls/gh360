from setuptools import setup

package_name = 'gh360_examples'

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
    maintainer='laurenz',
    maintainer_email='laurenz.elstner@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'monitor = gh360_examples.monitor:main',
            'streamdeck_test = gh360_examples.test_stream_deck:main',
            'door_monitor_control = gh360_examples.door_motor_control:main',
            'eef_pos_in_world = gh360_examples.eef_pos_in_world:main',
            'handle_sensor_filter = gh360_examples.handle_sensor_filter:main',
        ],
    },
)
