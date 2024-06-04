import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'helix_gripper'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch/*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sebtiburzio',
    maintainer_email='sebtiburzio@gmail.com',
    description='Control interface for the Helix gripper',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'helix_gripper_node = helix_gripper.helix_gripper:main'
        ],
    },
)
