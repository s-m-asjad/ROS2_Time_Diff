from setuptools import setup

package_name = 'time_diff'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
