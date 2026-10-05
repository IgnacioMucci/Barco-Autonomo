# Prueba del enlace companion <-> ground: SOLO MAVROS + nodo de enlace.
# No arranca nodos que arman o mandan PWM/velocidad (a diferencia de control_asv_launch.py).
#
#   ros2 launch control_asv enlace_ground_launch.py
#   ros2 launch control_asv enlace_ground_launch.py fcu_url:=udp://192.168.0.101:14550@ gcs_url:=/dev/ttyUSB3:57600
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('fcu_url', default_value='/dev/ttyACM0:57600',
                              description='Enlace a la Pixhawk (USB o udp://IP:14550@ por ethernet)'),
        DeclareLaunchArgument('gcs_url', default_value='/dev/ttyUSB3:57600',
                              description='Radio hacia ground'),

        Node(
            package='mavros',
            executable='mavros_node',
            output='screen',
            parameters=[{
                'fcu_url': LaunchConfiguration('fcu_url'),
                'gcs_url': LaunchConfiguration('gcs_url'),
                'tgt_system': 1,
                'tgt_component': 1,
            }],
        ),

        Node(
            package='control_asv',
            executable='nodo_enlace_ground',
            name='enlace_ground',
            output='screen',
        ),
    ])

'/root/ros_ws/install/control_asv/lib/control_asv'