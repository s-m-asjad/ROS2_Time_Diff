from setuptools import setup
import os
from glob import glob

package_name = 'time_diff'
submodules = "time_diff/submodules"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch/'),glob('launch/*.launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='asjad',
    maintainer_email='s_m_asjad@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'data_publisher = time_diff.data_publisher:main',
            'data_subscriber = time_diff.data_subscriber:main'
        ],
    },
)
