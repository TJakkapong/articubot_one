from setuptools import find_packages, setup

package_name = 'laser_scan_merger'

setup(
    name='laser_scan_merger',
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'sensor_msgs'
    ],
    zip_safe=True,
    maintainer='kittawat',
    maintainer_email='kittawat@todo.todo',
    description='Merges two LaserScan topics into one.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'laser_scan_merger = laser_scan_merger.laser_scan_merger:main',
        ],
    },
)

