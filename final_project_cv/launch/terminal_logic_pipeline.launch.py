from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    object_diameter_m = LaunchConfiguration("object_diameter_m")
    fps = LaunchConfiguration("fps")

    return LaunchDescription([
        DeclareLaunchArgument("object_diameter_m", default_value="0.22"),
        DeclareLaunchArgument("fps", default_value="2.0"),
        Node(
            package="final_project_cv",
            executable="centroid_test_source",
            name="centroid_test_source",
            output="screen",
            parameters=[{
                "centroid_topic": "/target_centroid",
                "camera_info_topic": "/camera/camera_info",
                "frame_id": "odom",
                "fps": ParameterValue(fps, value_type=float),
                "width": 640,
                "height": 480,
                "fx": 554.0,
                "fy": 554.0,
                "log_throttle_sec": 1.5,
            }],
        ),
        Node(
            package="final_project_cv",
            executable="target_localizer",
            name="target_localizer",
            output="screen",
            parameters=[{
                "centroid_topic": "/target_centroid",
                "camera_info_topic": "/camera/camera_info",
                "target_frame": "odom",
                "camera_frame": "odom",
                "object_diameter_m": ParameterValue(object_diameter_m, value_type=float),
                "max_range_m": 20.0,
                "log_throttle_sec": 1.5,
            }],
        ),
        Node(
            package="final_project_cv",
            executable="target_trace_recorder",
            name="target_trace_recorder",
            output="screen",
            parameters=[{
                "centroid_topic": "/target_centroid",
                "target_point_topic": "/target_point_odom",
                "output_dir": "/root/ros2_ws/src/final_project_cv/output",
                "save_plot_every_sec": 2.0,
            }],
        ),
    ])
