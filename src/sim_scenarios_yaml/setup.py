import os
from glob import glob
from setuptools import setup

package_name = 'sim_scenarios_yaml'

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
        (os.path.join('share', package_name, 'scenarios'),
            glob('scenarios/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Leyde Calderón Sánchez',
    maintainer_email='leydecs17@gmail.com',
    description='Native YAML-driven scenario spawner, no scenario_runner required.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yaml_scenario = sim_scenarios_yaml.yaml_scenario_node:main',
        ],
    },
)
