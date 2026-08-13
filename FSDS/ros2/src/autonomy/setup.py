from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'autonomy'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name), ['README.md']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='satvara',
    maintainer_email='satvara@example.com',
    description='Local cone perception, planning, and control for FSDS.',
    license='GPL-2.0-only',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'cone_detector = autonomy.cone_detector:main',
            'local_planner = autonomy.local_planner:main',
            'pure_pursuit = autonomy.pure_pursuit:main',
        ],
    },
)
