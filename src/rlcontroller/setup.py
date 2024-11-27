import os
from glob import glob
from setuptools import setup

package_name = 'rlcontroller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*_launch.py')),
        (os.path.join('share', package_name), glob('config/policies/tag_1/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='suntao',
    maintainer_email='suntao.hn@gmail.com',
    description='RL-based controller for ambot robot (qaudruped robot)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rlcontroller_node = rlcontroller.ambotw1_run:main',
            'rlcontroller_visual_node = rlcontroller.ambotw1_visual:main',
        ],
    },
)
