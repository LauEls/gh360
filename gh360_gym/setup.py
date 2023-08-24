from setuptools import setup, find_packages
import sys, os.path

package_name = 'gh360_gym'

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'gh360_gym'))

setup(
    name=package_name,
    version='0.0.1',
    # packages=[package_name],
    packages=[package for package in find_packages()
                if package.startswith('gh360_gym')],
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
            # 'door_env = gh360_gym.envs.door:main',
            'talker = gh360_gym.envs.test_pub:main',
        ],
    },
)
