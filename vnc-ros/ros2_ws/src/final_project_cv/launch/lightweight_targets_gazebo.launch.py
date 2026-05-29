from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_world = PathJoinSubstitution(
        [FindPackageShare("final_project_cv"), "worlds", "lightweight_targets.world"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=default_world,
                description="Gazebo world used for lightweight target/camera testing.",
            ),
            ExecuteProcess(
                cmd=["gazebo", "--verbose", LaunchConfiguration("world")],
                output="screen",
            ),
        ]
    )
