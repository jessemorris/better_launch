from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution


def generate_launch_description():

    colors = {
        'background_r': '200'
    }

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('better_launch'),
                    'examples',
                    '08_better_turtlesim.launch.py'
                ])
            ]),
            launch_arguments={
                'turtlesim_ns': 'turtlesim4',
                'use_provided_red': 'False',
                'new_background_r': TextSubstitution(text=str(colors['background_r'])),
                'remap': "/turtlesim4/turtle1/cmd_vel@cmd_vel,topic1@topic2"
            }.items()
        )
    ])