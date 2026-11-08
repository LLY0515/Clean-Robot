import os
from glob import glob

from setuptools import setup

package_name = 'coverage_mission'

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
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lunyi',
    maintainer_email='lunyi@todo.todo',
    description='Coverage mission executor on top of opennav_coverage',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'coverage_executor = coverage_mission.coverage_executor:main',
            'extract_coverage_area = coverage_mission.area_extractor:main',
        ],
    },
)
