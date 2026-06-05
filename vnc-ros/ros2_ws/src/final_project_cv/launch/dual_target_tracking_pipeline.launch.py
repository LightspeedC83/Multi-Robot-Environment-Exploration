from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def detector_node(name, target, centroid_topic, debug_image_topic):
    return Node(
        package="final_project_cv",
        executable="vision_target_detector",
        name=name,
        # namespace is the simple trick that lets robot1 and robot2 run side by side
        namespace=LaunchConfiguration("robot_namespace"),
        output="screen",
        parameters=[{
            "image_topic": LaunchConfiguration("image_topic"),
            "target": target,
            "centroid_topic": centroid_topic,
            "debug_image_topic": debug_image_topic,
            "use_yolo": ParameterValue(LaunchConfiguration("use_yolo"), value_type=bool),
            "use_fastsam": ParameterValue(LaunchConfiguration("use_fastsam"), value_type=bool),
            "fastsam_weights": LaunchConfiguration("fastsam_weights"),
            "fastsam_conf": ParameterValue(LaunchConfiguration("fastsam_conf"), value_type=float),
            "fastsam_iou": ParameterValue(LaunchConfiguration("fastsam_iou"), value_type=float),
            "yolo_conf": ParameterValue(LaunchConfiguration("yolo_conf"), value_type=float),
            "process_width": ParameterValue(LaunchConfiguration("process_width"), value_type=int),
            "process_every_n": ParameterValue(LaunchConfiguration("process_every_n"), value_type=int),
            "imgsz": ParameterValue(LaunchConfiguration("imgsz"), value_type=int),
            "selection_strategy": "bottom" if target == "sports ball" else "largest",
            "smooth_alpha": ParameterValue(LaunchConfiguration("smooth_alpha"), value_type=float),
            "disable_nnpack": True,
            "fuse_yolo_model": False,
            "display_classes": "bottle,sports ball",
        }],
    )


def localizer_node(name, target, object_diameter_m, centroid_topic, point_topic, pose_topic, prefer_lidar_range):
    return Node(
        package="final_project_cv",
        executable="target_localizer",
        name=name,
        namespace=LaunchConfiguration("robot_namespace"),
        output="screen",
        parameters=[{
            "centroid_topic": centroid_topic,
            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
            "scan_topic": LaunchConfiguration("scan_topic"),
            "point_topic": point_topic,
            "pose_topic": pose_topic,
            "target": target,
            "target_frame": LaunchConfiguration("target_frame"),
            "camera_frame": LaunchConfiguration("camera_frame"),
            "lidar_frame": LaunchConfiguration("lidar_frame"),
            "object_diameter_m": ParameterValue(object_diameter_m, value_type=float),
            "prefer_lidar_range": prefer_lidar_range,
            "lidar_window_deg": ParameterValue(LaunchConfiguration("lidar_window_deg"), value_type=float),
            "min_lidar_target_range_m": ParameterValue(LaunchConfiguration("min_lidar_target_range_m"), value_type=float),
            "lidar_vision_tolerance_m": ParameterValue(LaunchConfiguration("lidar_vision_tolerance_m"), value_type=float),
            "lidar_vision_tolerance_ratio": ParameterValue(LaunchConfiguration("lidar_vision_tolerance_ratio"), value_type=float),
            "log_throttle_sec": ParameterValue(LaunchConfiguration("log_throttle_sec"), value_type=float),
        }],
    )


def simulation_camera_tf_node():
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="sim_camera_tf",
        namespace=LaunchConfiguration("robot_namespace"),
        arguments=[
            "--x", "0.19",
            "--y", "0.0",
            "--z", "0.17",
            "--roll", "-1.30899693899",
            "--pitch", "0.0",
            "--yaw", "-1.57079632679",
            "--frame-id", LaunchConfiguration("base_frame"),
            "--child-frame-id", LaunchConfiguration("camera_frame"),
        ],
    )


def simulation_lidar_tf_node():
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="sim_lidar_tf",
        namespace=LaunchConfiguration("robot_namespace"),
        arguments=[
            "--x", "0.05",
            "--y", "0.0",
            "--z", "0.18",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", LaunchConfiguration("base_frame"),
            "--child-frame-id", LaunchConfiguration("lidar_frame"),
        ],
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_namespace", default_value="robot1"),
        DeclareLaunchArgument("image_topic", default_value="camera/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="camera/camera_info"),
        DeclareLaunchArgument("scan_topic", default_value="scan"),
        #these frame defaults follow robot_namespace, so robot2 only needs one launch arg
        DeclareLaunchArgument("target_frame", default_value=[LaunchConfiguration("robot_namespace"), "/odom"]),
        DeclareLaunchArgument("base_frame", default_value=[LaunchConfiguration("robot_namespace"), "/base_link"]),
        DeclareLaunchArgument("camera_frame", default_value=[LaunchConfiguration("robot_namespace"), "/camera_link"]),
        DeclareLaunchArgument("lidar_frame", default_value=[LaunchConfiguration("robot_namespace"), "/base_scan"]),
        DeclareLaunchArgument("yolo_conf", default_value="0.01"),
        DeclareLaunchArgument("use_yolo", default_value="true"),
        DeclareLaunchArgument("use_fastsam", default_value="true"),
        DeclareLaunchArgument("fastsam_weights", default_value="FastSAM-s.pt"),
        DeclareLaunchArgument("fastsam_conf", default_value="0.4"),
        DeclareLaunchArgument("fastsam_iou", default_value="0.9"),
        DeclareLaunchArgument("process_width", default_value="640"),
        DeclareLaunchArgument("process_every_n", default_value="1"),
        DeclareLaunchArgument("imgsz", default_value="640"),
        DeclareLaunchArgument("smooth_alpha", default_value="0.65"),
        DeclareLaunchArgument("heuristic_diameter_m", default_value="0.34"),
        DeclareLaunchArgument("goal_diameter_m", default_value="0.15"),
        DeclareLaunchArgument("lidar_window_deg", default_value="4.0"),
        DeclareLaunchArgument("min_lidar_target_range_m", default_value="0.30"),
        DeclareLaunchArgument("lidar_vision_tolerance_m", default_value="0.75"),
        DeclareLaunchArgument("lidar_vision_tolerance_ratio", default_value="0.45"),
        DeclareLaunchArgument("log_throttle_sec", default_value="0.75"),
        simulation_camera_tf_node(),
        simulation_lidar_tf_node(),
        detector_node(
            "heuristic_bottle_detector",
            "bottle",
            "heuristic_centroid",
            "heuristic_debug_image",
        ),
        localizer_node(
            "heuristic_bottle_localizer",
            "bottle",
            LaunchConfiguration("heuristic_diameter_m"),
            "heuristic_centroid",
            "heuristic_point_odom",
            "heuristic_pose_odom",
            True,
        ),
        detector_node(
            "goal_sphere_detector",
            "sports ball",
            "goal_centroid",
            "goal_debug_image",
        ),
        localizer_node(
            "goal_sphere_localizer",
            "sports ball",
            LaunchConfiguration("goal_diameter_m"),
            "goal_centroid",
            "goal_point_odom",
            "goal_pose_odom",
            False,
        ),
    ])
