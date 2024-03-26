import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'helix_transmission'

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
    description='Transmission providing abstraction between Helix motors and tendons',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'tendon_transmission_node = helix_transmission.tendon_transmission:main'
        ],
    },
)
