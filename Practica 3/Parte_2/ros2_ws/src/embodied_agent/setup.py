from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'embodied_agent'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world') + glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Raúl Boix',
    maintainer_email='pepetrola2015@gmail.com',
    description='Embodied robot agent for TurtleBot 4 using Gemini multimodal LLM',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'audio_in_node = embodied_agent.audio_in_node:main',
            'brain_node = embodied_agent.brain_node:main',
            'speech_node = embodied_agent.speech_node:main',
            'action_executor_node = embodied_agent.action_executor_node:main',
        ],
    },
)
