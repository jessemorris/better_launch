from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
)


def generate_launch_description():
    # See https://github.com/ros2/launch_ros/blob/rolling/ros2launch/ros2launch/api/api.py#L175
    ros2_include = IncludeLaunchDescription(
        AnyLaunchDescriptionSource("/opt/ros/humble/share/composition/launch/composition_demo.launch.py")
    )

    return LaunchDescription([ros2_include])
