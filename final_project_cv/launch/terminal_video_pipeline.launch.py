from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    source = LaunchConfiguration("source")
    target = LaunchConfiguration("target")
    object_diameter_m = LaunchConfiguration("object_diameter_m")
    use_fastsam = LaunchConfiguration("use_fastsam")
    yolo_conf = LaunchConfiguration("yolo_conf")
    process_width = LaunchConfiguration("process_width")
    process_every_n = LaunchConfiguration("process_every_n")
    imgsz = LaunchConfiguration("imgsz")
    fps = LaunchConfiguration("fps")

    return LaunchDescription([
        DeclareLaunchArgument("source", default_value="/root/ros2_ws/src/final_project_cv/scripts/ball.mp4"),
        DeclareLaunchArgument("target", default_value="sports ball"),
        DeclareLaunchArgument("object_diameter_m", default_value="0.22"),
        DeclareLaunchArgument("use_fastsam", default_value="true"),
        DeclareLaunchArgument("yolo_conf", default_value="0.20"),
        DeclareLaunchArgument("process_width", default_value="416"),
        DeclareLaunchArgument("process_every_n", default_value="2"),
        DeclareLaunchArgument("imgsz", default_value="416"),
        DeclareLaunchArgument("fps", default_value="8.0"),
        Node(
            package="final_project_cv",
            executable="video_source",
            name="video_source",
            output="screen",
            parameters=[{
                "source": source,
                "image_topic": "/camera/image_raw",
                "camera_info_topic": "/camera/camera_info",
                "frame_id": "odom",
                "fps": ParameterValue(fps, value_type=float),
            }],
        ),
        Node(
            package="final_project_cv",
            executable="vision_target_detector",
            name="vision_target_detector",
            output="screen",
            parameters=[{
                "image_topic": "/camera/image_raw",
                "target": target,
                "centroid_topic": "/target_centroid",
                "debug_image_topic": "/target_debug_image",
                "use_fastsam": ParameterValue(use_fastsam, value_type=bool),
                "yolo_conf": ParameterValue(yolo_conf, value_type=float),
                "process_width": ParameterValue(process_width, value_type=int),
                "process_every_n": ParameterValue(process_every_n, value_type=int),
                "imgsz": ParameterValue(imgsz, value_type=int),
                "selection_strategy": "largest",
                "smooth_alpha": 0.65,
                "publish_debug_image": False,
                "disable_nnpack": True,
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
            }],
        ),
    ])
