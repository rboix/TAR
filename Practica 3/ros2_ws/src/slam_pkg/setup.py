from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'slam_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'models', 'maze_2'), glob('models/maze_2/*')),
        (os.path.join('share', package_name, 'models', 'obstacles'), glob('models/obstacles/*')),
        (os.path.join('share', package_name, 'models', 'ejer2'), glob('models/ejer2/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='davidetsm-docker',
    maintainer_email='davidetsm-docker@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
