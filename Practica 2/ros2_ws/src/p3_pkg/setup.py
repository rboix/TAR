from setuptools import setup
import os
from glob import glob

package_name = 'p3_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'models', 'parking'), glob('models/parking/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alumno',
    maintainer_email='alumno@universidad.es',
    description='Paquete ROS 2 para movimientos del robot (P3)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'movimiento = p3_pkg.movimiento:main',
            'dibuja_mov = p3_pkg.dibuja_mov:main',
            'repetir_mov = p3_pkg.repetir_mov:main',
            'aparcamiento = p3_pkg.aparcamiento:main',
        ],
    },
)
