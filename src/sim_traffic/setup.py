import os
from glob import glob
from setuptools import setup

package_name = 'sim_traffic'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Leyde Calderón Sánchez',
    maintainer_email='leydecs17@gmail.com',
    description='Seeded, reproducible background traffic for a CARLA scenario.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'seeded_traffic = sim_traffic.seeded_traffic_node:main',
        ],
    },
)
