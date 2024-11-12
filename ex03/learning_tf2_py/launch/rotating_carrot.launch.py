import os

from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    demo_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('learning_tf2_py'), 'launch'),
            '/turtle_tf2_demo.launch.py']),
        )

    return LaunchDescription([
        demo_nodes,

        DeclareLaunchArgument(
            'radius', default_value='1.0',
            description='Distance between carrot1 and turtle1.'
        ),

        DeclareLaunchArgument(
            'direction_of_rotation', default_value='1',
            description='Direction in which carrot1 is rotating(1 for clockwise, -1 for counter-clockwise.)'
        ),

        Node(
            package='learning_tf2_py',
            executable='rotating_carrot',
            name='carrot',
            parameters= [
                {'radius': ParameterValue(LaunchConfiguration('radius'), value_type=float)},
                {'direction_of_rotation': ParameterValue(LaunchConfiguration('direction_of_rotation'), value_type=int)}
            ]
        ),
    ])