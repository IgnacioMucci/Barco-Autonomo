from setuptools import find_packages, setup

package_name = 'mavlink_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='asv',
    maintainer_email='asv@example.com',
    description='Heartbeat MAVLink a Mission Planner con pymavlink',
    license='MIT',
    entry_points={
        'console_scripts': [
            'heartbeat_node = mavlink_bridge.heartbeat_node:main',
        ],
    },
)
