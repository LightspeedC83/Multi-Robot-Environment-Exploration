from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def mapper_process(robot_namespace):
    robot_prefix = f"{robot_namespace}/"
    return ExecuteProcess(
        cmd=[
            "python3",
            LaunchConfiguration("mapper_script"),
            "--ros-args",
            "-r", f"__ns:=/{robot_namespace}",
            "-r", f"__node:=world_mapper_{robot_namespace}",
            "-p", "cmd_vel_topic:=cmd_vel",
            "-p", "scan_topic:=scan",
            "-p", "odom_topic:=odom",
            "-p", f"odom_frame:={robot_prefix}odom",
            "-p", f"base_frame:={robot_prefix}base_link",
            "-p", f"laser_frame:={robot_prefix}base_scan",
            "-p", f"map_frame:={robot_prefix}odom",
        ],
        output="screen",
    )


def coordinator_process():
    return ExecuteProcess(
        cmd=[
            "python3",
            LaunchConfiguration("coordinator_script"),
            "--ros-args",
            "-p",
            [
                "min_exploration_before_goal_sec:=",
                LaunchConfiguration("min_exploration_before_goal_sec"),
            ],
        ],
        output="screen",
    )


def cleanup_previous_demo():
    return ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            (
                "python3 /root/ros2_ws/src/final_project_cv/tools/cleanup_demo_processes.py --quiet; "
                "/opt/ros/humble/bin/ros2 daemon stop || true; "
                "rm -f /root/.gazebo/gui.ini"
            ),
        ],
        condition=IfCondition(LaunchConfiguration("fresh_start")),
        output="screen",
    )


def gazebo_top_down_camera_reset():
    return ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            (
                "timeout 1 gz topic "
                "-t /gazebo/lightweight_targets/user_camera/pose "
                "-m gazebo.msgs.Pose "
                "-p 'position { x: 0 y: 0 z: 8 } "
                "orientation { x: 0 y: 0.7071068 z: 0 w: 0.7071068 }' || true"
            ),
        ],
        output="screen",
    )


def map_merger_node():
    return Node(
        package="merger",
        executable="map_merger_node",
        name="map_coordinator",
        output="screen",
        parameters=[{
            "confidence_threshold": ParameterValue(
                LaunchConfiguration("confidence_threshold"),
                value_type=float,
            ),
            "map_topic_template": "/SLAM_map_{id}",
            "map_topic_alias_templates": ["/{robot_id}/SLAM_map"],
        }],
    )


def demo_finalizer_node():
    return Node(
        package="final_project_cv",
        executable="demo_finalizer",
        name="demo_finalizer",
        output="screen",
        condition=IfCondition(LaunchConfiguration("auto_finalize")),
        parameters=[{
            "results_dir": "/root/ros2_ws/src/final_path_results",
            "tools_dir": "/root/ros2_ws/src/final_project_cv/tools",
            "snapshot_seconds": ParameterValue(
                LaunchConfiguration("finalizer_snapshot_seconds"),
                value_type=float,
            ),
            "shutdown_on_complete": ParameterValue(
                LaunchConfiguration("shutdown_after_finalize"),
                value_type=bool,
            ),
        }],
    )


def cv_pipeline(robot_namespace, pipeline_launch):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pipeline_launch),
        launch_arguments={
            "robot_namespace": robot_namespace,
            "target_frame": f"{robot_namespace}/odom",
            "base_frame": f"{robot_namespace}/base_link",
            "camera_frame": f"{robot_namespace}/camera_link",
            "lidar_frame": f"{robot_namespace}/base_scan",
            "use_yolo": LaunchConfiguration("use_yolo"),
            "use_fastsam": LaunchConfiguration("use_fastsam"),
            "process_every_n": LaunchConfiguration("process_every_n"),
        }.items(),
    )


def generate_launch_description():
    gazebo_launch = PathJoinSubstitution([
        FindPackageShare("final_project_cv"),
        "launch",
        "lightweight_targets_gazebo.launch.py",
    ])
    pipeline_launch = PathJoinSubstitution([
        FindPackageShare("final_project_cv"),
        "launch",
        "dual_target_tracking_pipeline.launch.py",
    ])
    finalizer = demo_finalizer_node()

    return LaunchDescription([
        DeclareLaunchArgument(
            "mapper_script",
            default_value="/root/ros2_ws/src/mapper/mapper.py",
            description="Direct path to the mapper script mounted into the ROS container.",
        ),
        DeclareLaunchArgument(
            "coordinator_script",
            default_value="/root/ros2_ws/src/mapper/coordinator.py",
            description="Direct path to the planner/coordinator script mounted into the ROS container.",
        ),
        DeclareLaunchArgument(
            "confidence_threshold",
            default_value="0.5",
            description="Map-fusion confidence threshold.",
        ),
        DeclareLaunchArgument(
            "use_yolo",
            default_value="false",
            description="Use YOLO weights instead of synthetic-color fallback detection.",
        ),
        DeclareLaunchArgument(
            "use_fastsam",
            default_value="false",
            description="Enable FastSAM mask refinement for CV target detection.",
        ),
        DeclareLaunchArgument(
            "process_every_n",
            default_value="2",
            description="Run CV on every nth camera frame.",
        ),
        DeclareLaunchArgument(
            "fresh_start",
            default_value="true",
            description="Kill stale Gazebo/demo nodes before loading the world so robot poses reset.",
        ),
        DeclareLaunchArgument(
            "min_exploration_before_goal_sec",
            default_value="45.0",
            description="Hold early goal detections this long so frontier exploration is visible before final A* stop.",
        ),
        DeclareLaunchArgument(
            "auto_finalize",
            default_value="true",
            description="Capture report evidence, generate visuals, and end the launch after /mission_complete.",
        ),
        DeclareLaunchArgument(
            "finalizer_snapshot_seconds",
            default_value="5.0",
            description="Seconds to collect final map and CV snapshots after /mission_complete.",
        ),
        DeclareLaunchArgument(
            "shutdown_after_finalize",
            default_value="true",
            description="End the integrated launch after final artifacts are generated.",
        ),
        #  gazebo reloading: old GUI sessions keep model poses, so clean them before a new take.
        cleanup_previous_demo(),
        TimerAction(period=1.0, actions=[
            IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo_launch)),
        ]),
        TimerAction(period=3.0, actions=[
            gazebo_top_down_camera_reset(),
        ]),
        TimerAction(period=5.5, actions=[
            gazebo_top_down_camera_reset(),
        ]),
        TimerAction(period=1.5, actions=[
            coordinator_process(),
            map_merger_node(),
            finalizer,
        ]),
        TimerAction(period=2.2, actions=[
            mapper_process("robot1"),
        ]),
        TimerAction(period=2.6, actions=[
            mapper_process("robot2"),
        ]),
        TimerAction(period=3.2, actions=[
            cv_pipeline("robot1", pipeline_launch),
            cv_pipeline("robot2", pipeline_launch),
        ]),
        RegisterEventHandler(
            OnProcessExit(
                target_action=finalizer,
                on_exit=[
                    EmitEvent(event=Shutdown(reason="integrated demo finalized")),
                ],
            ),
            condition=IfCondition(LaunchConfiguration("auto_finalize")),
        ),
    ])
