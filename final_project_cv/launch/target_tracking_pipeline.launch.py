from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    target = LaunchConfiguration("target")
    object_diameter_m = LaunchConfiguration("object_diameter_m")
    image_topic = LaunchConfiguration("image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    target_frame = LaunchConfiguration("target_frame")
    camera_frame = LaunchConfiguration("camera_frame")
    yolo_conf = LaunchConfiguration("yolo_conf")
    use_fastsam = LaunchConfiguration("use_fastsam")
    fastsam_weights = LaunchConfiguration("fastsam_weights")
    fastsam_conf = LaunchConfiguration("fastsam_conf")
    fastsam_iou = LaunchConfiguration("fastsam_iou")
    process_width = LaunchConfiguration("process_width")
    process_every_n = LaunchConfiguration("process_every_n")
    imgsz = LaunchConfiguration("imgsz")
    smooth_alpha = LaunchConfiguration("smooth_alpha")

    return LaunchDescription([
        DeclareLaunchArgument("target", default_value="bottle"),
        DeclareLaunchArgument("object_diameter_m", default_value="0.07"),
        DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera_info"),
        DeclareLaunchArgument("target_frame", default_value="odom"),
        DeclareLaunchArgument("camera_frame", default_value=""),
        DeclareLaunchArgument("yolo_conf", default_value="0.25"),
        DeclareLaunchArgument("use_fastsam", default_value="true"),
        DeclareLaunchArgument("fastsam_weights", default_value="FastSAM-s.pt"),
        DeclareLaunchArgument("fastsam_conf", default_value="0.4"),
        DeclareLaunchArgument("fastsam_iou", default_value="0.9"),
        DeclareLaunchArgument("process_width", default_value="640"),
        DeclareLaunchArgument("process_every_n", default_value="1"),
        DeclareLaunchArgument("imgsz", default_value="640"),
        DeclareLaunchArgument("smooth_alpha", default_value="0.65"),
        Node(
            package="final_project_cv",
            executable="vision_target_detector",
            name="vision_target_detector",
            output="screen",
            parameters=[{
                "image_topic": image_topic,
                "target": target,
                "centroid_topic": "/target_centroid",
                "use_fastsam": ParameterValue(use_fastsam, value_type=bool),
                "fastsam_weights": fastsam_weights,
                "fastsam_conf": ParameterValue(fastsam_conf, value_type=float),
                "fastsam_iou": ParameterValue(fastsam_iou, value_type=float),
                "yolo_conf": ParameterValue(yolo_conf, value_type=float),
                "process_width": ParameterValue(process_width, value_type=int),
                "process_every_n": ParameterValue(process_every_n, value_type=int),
                "imgsz": ParameterValue(imgsz, value_type=int),
                "selection_strategy": "largest",
                "smooth_alpha": ParameterValue(smooth_alpha, value_type=float),
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
                "camera_info_topic": camera_info_topic,
                "target_frame": target_frame,
                "camera_frame": camera_frame,
                "object_diameter_m": ParameterValue(object_diameter_m, value_type=float),
            }],
        ),
    ])
