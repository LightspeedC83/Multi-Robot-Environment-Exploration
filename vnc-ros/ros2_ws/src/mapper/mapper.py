#!/usr/bin/env python
#The line above is important so that this file is interpreted with Python when running it.

# Multi-robot mapper/controller node.

# Import of python modules.
import math # use of pi.
import random # use for generating a random real number
from enum import Enum
import time
import functools
import numpy as np # for map grid representations and operations
from anytree import Node as TreeNode # for search algorithms (so important to import as Tree node because we have a Node class from ros2 already
import heapq # for A*

# import of relevant libraries.
import rclpy # module for ROS APIs
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.clock import Clock, ClockType
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist # message type for cmd_vel
from sensor_msgs.msg import LaserScan # message type for scan
from nav_msgs.msg import OccupancyGrid, Odometry # message type for occupancyGrid
from  nav_msgs.msg import MapMetaData # for the slam_map msg.info
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Point, Quaternion # for the ifnromation stored in slam_map msg.info and a lot of communication with coordinator
from tf2_ros import TransformListener, Buffer

from std_msgs.msg import Bool # for id_active publisher

from std_srvs.srv import SetBool # service type 

# importing custom services
from mapper_interfaces.srv import GetUniqueID
from mapper_interfaces.srv import GetNewFrontierPath

# Constants.
# Topic names
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_SCAN_TOPIC = 'base_scan' # name of laserscan topic for Stage simulator. For Gazebo, 'scan'
OCCUPANCY_GRID_TOPIC = 'nav_msgs/OccupancyGrid'
DEFAULT_SERVICE_NAME = 'on_off'
DEFAULT_ODOM_FRAME = 'rosbot/odom'
DEFAULT_BASE_FRAME = 'rosbot/base_link'
DEFAULT_LASER_FRAME = 'rosbot/laser'

# Frequency at which the loop operates
FREQUENCY = 100 #Hz.

# Parameters
LINEAR_VELOCITY = 0.5 # m/s
ANGULAR_VELOCITY = 0.55 # rad/s

SLAM_MAP_RESOLUTION_SCALAR = 10 # number of cells in the SLAM map per meter

# Implementation of Extra Credit
PROBABILISTIC_MAPPING =  True # if this is true, instead of using binary updates, we Implement a recursive Bayesian update using log-odds to make the map resilient to sensor noise.
CELL_PROBABILITY_OCCUPIED = 70 # the minimum probability a cell has to have to be considered occupied

SCAN_DOWNSAMPLING = 1 # the robot will process every nth scan (this value is n) so 1=process every scan, 2=process every other scan...
SMOOTHING_KERNEL_SIZE = 10  # the kernel size applied to the gaussian smoothing algorithm
SMOOTHING_SIGMA = 6 # The standard deviation applied to the gaussian smoothing algorithm
MAP_CLEAR_THRESHOLD = 33 # program treats any value below this as free space
MAP_OCCUPIED_THRESHOLD = 80 # program treats any value above this as occupied and to be avoided

PROTECTION_RADIUS = 0.3 # [m] The radius within which the robot will stop if it detects something, then it will wait and find a new path to nearest frontier
FORWARD_PROTECTION_ANGLE_RAD = math.radians(45)
EMERGENCY_PROTECTION_RADIUS = 0.12 # [m]
CLEARANCE_TARGET_RADIUS = 0.46 # [m] normal LiDAR spacing we try to preserve while driving past objects
CLEARANCE_SOFT_RADIUS = 0.68 # [m] begin steering away before the emergency layer has to fire
CLEARANCE_HARD_STOP_RADIUS = 0.24 # [m] stop forward motion and turn when the front gap gets this tight
CLEARANCE_SCAN_STALE_SEC = 0.45 # [s]
CLEARANCE_FRONT_TURN = 0.28 # [rad/s]
CLEARANCE_FRONT_GAIN = 0.75
CLEARANCE_SIDE_GAIN = 0.65
CLEARANCE_MAX_TURN = 0.62 # [rad/s]
CLEARANCE_MIN_LINEAR = 0.06 # [m/s]
LASER_SLEEP_TIME_AFTER_INTERRUPT = 2 # [s] short cooldown after local escape, not a minute-long trap
ESCAPE_BACKUP_TIME = 0.8 # [s]
ESCAPE_TURN_TIME = 1.1 # [s]
ESCAPE_BACKUP_VELOCITY = -0.18 # [m/s]
ESCAPE_FORWARD_VELOCITY = 0.16 # [m/s]
ESCAPE_TURN_VELOCITY = 0.35 # [rad/s]
STUCK_PROGRESS_EPS = 0.03 # [m]
STUCK_WATCHDOG_SEC = 2.2 # [s]
MISSION_STOP_BURST_SEC = 3.0 # [s] repeated zero commands after final answer latching

USE_SIM_TIME = True
STARTUP_TIMEOUT = 1.0 # s. Max wait for simulator/controller startup.

class fsm(Enum):
    OFF = 0
    ON = 1
    WAITING_FOR_PATH = 2
    EXECUTING_PATH = 3
    RECOVERY = 4

class recovery_fsm(Enum):
    FINE = 0
    WAITING_FOR_PATH = 1
    EXECUTING_PATH = 2
    ESCAPING = 3

class local_fsm(Enum):
    IDLE = 0
    ROTATING = 1
    MOVING = 2


NEIGHBOR_LIST = [
        (-1,1),  (0,1),  (1,1),
        (-1,0),          (1,0),
        (-1,-1), (0,-1), (1,-1)
        ]


class WorldMapper(Node):
    def __init__(self, 
                 LINEAR_VELOCITY=LINEAR_VELOCITY,
                 ANGULAR_VELOCITY=ANGULAR_VELOCITY,               
                 PROBABILISTIC_MAPPING=PROBABILISTIC_MAPPING,
                 SCAN_DOWNSAMPLING=SCAN_DOWNSAMPLING,
                 SLAM_MAP_RESOLUTION_SCALAR=SLAM_MAP_RESOLUTION_SCALAR,
                 PROTECTION_RADIUS=PROTECTION_RADIUS, 
                 node_name="world_mapper", 
                 context=None):
        """Constructor."""
        super().__init__(node_name, context=context)

        # Workaround not to use roslaunch
        use_sim_time_param = rclpy.parameter.Parameter(
            'use_sim_time',
            rclpy.Parameter.Type.BOOL,
            USE_SIM_TIME
        )
        self.set_parameters([use_sim_time_param])

        self.declare_parameter("cmd_vel_topic", DEFAULT_CMD_VEL_TOPIC)
        self.declare_parameter("scan_topic", DEFAULT_SCAN_TOPIC)
        self.declare_parameter("odom_topic", "")
        self.declare_parameter("odom_frame", DEFAULT_ODOM_FRAME)
        self.declare_parameter("base_frame", DEFAULT_BASE_FRAME)
        self.declare_parameter("laser_frame", DEFAULT_LASER_FRAME)
        self.declare_parameter("map_frame", DEFAULT_ODOM_FRAME)
        self.declare_parameter("id_service_name", "/get_unique_id")
        self.declare_parameter("path_service_name", "/get_path")
        self.declare_parameter("map_topic_template", "/SLAM_map_{id}")
        self.declare_parameter("pose_topic_template", "/pose_{id}")
        self.declare_parameter("path_topic_template", "/nav_path_{id}")
        self.declare_parameter("active_topic_template", "/id_active_{id}")
        self.declare_parameter("mission_complete_topic", "/mission_complete")
        self.declare_parameter("stop_on_mission_complete", True)

        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.scan_topic = self.get_parameter("scan_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.laser_frame = self.get_parameter("laser_frame").value
        self.map_frame = self.get_parameter("map_frame").value or self.odom_frame
        self.id_service_name = self.get_parameter("id_service_name").value
        self.path_service_name = self.get_parameter("path_service_name").value
        self.map_topic_template = self.get_parameter("map_topic_template").value
        self.pose_topic_template = self.get_parameter("pose_topic_template").value
        self.path_topic_template = self.get_parameter("path_topic_template").value
        self.active_topic_template = self.get_parameter("active_topic_template").value
        self.mission_complete_topic = self.get_parameter("mission_complete_topic").value
        self.stop_on_mission_complete = self.get_parameter("stop_on_mission_complete").value

        ## getting this robot's ID ##
        self.id_client = self.create_client(GetUniqueID, self.id_service_name)
        self.id = self.request_id("mapper")
        self.robot_id_name = f"robot{self.id}" if self.id is not None else "robot_NO_ID"

        ## setting up service clients ##
        self.nav_path_client = self.create_client(GetNewFrontierPath, self.path_service_name) # getting path from coordinator
        self.nav_path_listener = self.create_subscription(PoseArray, self._topic_from_template(self.path_topic_template), self._nav_path_callback, 1)
        self._mission_complete_sub = self.create_subscription(Bool, self.mission_complete_topic, self._mission_complete_callback, 1)
        self.waiting_for_path_request = False

        ### Setting up publishers/subscribers. ###

        # Setting up the publisher to send velocity commands.
        self._cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 1)

        # Setting up subscriber receiving messages from the laser.
        self._laser_sub = self.create_subscription(LaserScan, self.scan_topic, self._laser_callback, 1)
        self.latest_odom_msg = None
        if self.odom_topic:
            self._odom_sub = self.create_subscription(Odometry, self.odom_topic, self._odom_callback, 10)
        else:
            self._odom_sub = None

        # Setting up a publisher to publish the map data the robot creates
        if not self.id is None:
            self.map_publish_topic_name = self._topic_from_template(self.map_topic_template)
        else:
            self.map_publish_topic_name = '/SLAM_map_NO_ID'   
            self._fsm = fsm.OFF # turning off the robot 
        
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        #  map remembering: RViz usually joins late, so keep the newest grid available.
        self._SLAM_map_pub = self.create_publisher(OccupancyGrid, self.map_publish_topic_name, map_qos)
        
        # setting up a publisher to publish the current position of the robot
        if not self.id is None:
            self.pose_publish_topic_name = self._topic_from_template(self.pose_topic_template)
        else:
            self.pose_publish_topic_name = '/pose_NO_ID'  

        self._pose_pub = self.create_publisher(PoseStamped, self.pose_publish_topic_name, 1)
        
        # setting up a publisher to publish whether this ID is active
        self._id_active_pub = self.create_publisher(Bool, self._topic_from_template(self.active_topic_template), 1)
        
        id_active_msg = Bool()
        id_active_msg.data = True
        self._id_active_pub.publish(id_active_msg) # publishing that this node is active!



        # setting up on/off service
        self._on_off_service = self.create_service(SetBool, f'{node_name}/{DEFAULT_SERVICE_NAME}', self._turn_on_off_callback)
        self._fsm = fsm.ON
        self._local_fsm = local_fsm.IDLE

         
        #### parameter definitions ####

        # Everything for the SLAM map will be initialized based on the sensor range on the first sensor call
        self.SLAM_map_initialized = False
        self.SLAM_map_populated = False # we want to get our first goal point after we get data in the slam map, this contorls that 

        self.SLAM_MAP_RESOLUTION_SCALAR = SLAM_MAP_RESOLUTION_SCALAR # set by the user [cell/m]
        self.SLAM_map_res_m_per_cell = None # resolution of the slam map for odom coordinate conversion, set by initialization function [m/cell]

        self.SLAM_MAP_SIZE_X = None
        self.SLAM_MAP_SIZE_Y = None
        self.SLAM_map_origin_X = None # coordinates for the cell that odom's origin is in
        self.SLAM_map_origin_Y = None # coordinates for the cell that odom's origin is in
        self.SLAM_map_info = None # this holds information about the occupancy grid map we generate: .width and .height
        
        self.SLAM_map = None # initializing the slam map as all -1

        self.max_sensor_range = None # max range for the laser sensor [m]

        self.PROBABILISTIC_MAPPING=PROBABILISTIC_MAPPING # boolean for activation of implementation of extra credit
        self.SCAN_DOWNSAMPLING=SCAN_DOWNSAMPLING # robot processes every n scan
        self.scan_downsampling_count = self.SCAN_DOWNSAMPLING # value used to keep track of scans for skipping them

        self.nav_path = None # This is the path of points (in robot/odom coordinates) to the nearest frontier node
        self.last_received_nav_path = None
        self.last_nav_path_timestamp = rclpy.time.Time()
        self.next_path_retry_time = self.get_clock().now()
        self.next_path_retry_wall_time = 0.0
        self.awaiting_nav_path_after_success = False
        self.pose_warning_count = 0
        

        self.PROTECTION_RADIUS = PROTECTION_RADIUS
        self.done_time_for_laser_wait = self.get_clock().now()
        self._recovery_fsm = recovery_fsm.FINE
        self.recovery_path_executed = False
        self.recovery_path = None
        self.escape_stage = None
        self.escape_done_wall_time = 0.0
        self.escape_turn_direction = 1.0
        self.escape_linear_velocity = ESCAPE_BACKUP_VELOCITY
        self.motion_watchdog_pose = None
        self.motion_watchdog_wall_time = time.monotonic()
        self.latest_scan_msg = None
        self.latest_scan_wall_time = 0.0
        self.next_clearance_log_wall_time = 0.0

        # parameters for local level robot rotational and translational speed
        self.LINEAR_VELOCITY = LINEAR_VELOCITY
        self.ANGULAR_VELOCITY = ANGULAR_VELOCITY

        # parameters for controlling local motion of robot
        self.done = False # if the robot has mapped its whole environment
        self.mission_complete_logged = False
        self.mission_stop_until_wall_time = 0.0
        self.busy = False # if busy is true, then the robot is in the midst of an action
        self.action_done_time = self.get_clock().now() # this is the time at which the current action being excecuted will be finished
        self.next_motion_log_wall_time = 0.0

        # setting up transfer frames
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._mission_stop_timer = self.create_timer(0.1, self._mission_stop_timer_callback, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _topic_from_template(self, template):
        """Fill the mapper/coordinator topic templates for this assigned robot."""
        return str(template).format(id=self.id, robot_id=self.robot_id_name)

    def _odom_callback(self, msg):
        """Keep the latest Gazebo odometry around for pose publication and control."""
        self.latest_odom_msg = msg

    def _mission_complete_callback(self, msg):
        """Stop local exploration once the coordinator has a final demo path."""
        if not msg.data:
            return

        if not self.stop_on_mission_complete:
            #  demo continuing: the final path is published, but movement still looks better on video.
            if not self.mission_complete_logged:
                self.get_logger().info("Mission complete received; continuing exploration")
                self.mission_complete_logged = True
            return

        # The final path is already published by the coordinator; optional stopping keeps the robots still.
        self.done = True
        self.busy = False
        self.nav_path = None
        self._fsm = fsm.OFF
        self._local_fsm = local_fsm.IDLE
        self.mission_stop_until_wall_time = time.monotonic() + MISSION_STOP_BURST_SEC
        self.stop()
        if not self.mission_complete_logged:
            self.get_logger().info("Mission complete received; stopping exploration")
            self.mission_complete_logged = True

    def _mission_stop_timer_callback(self):
        """Keep final-stop commands alive long enough for Gazebo controllers to settle."""
        if self.mission_stop_until_wall_time <= 0.0:
            return

        if time.monotonic() > self.mission_stop_until_wall_time:
            self.mission_stop_until_wall_time = 0.0
            return

        #  Stop holding: late control ticks or buffered commands should lose to final answer.
        self.stop()


    def request_id(self, requester_name):
        """this is a a function that gets the id of this robot assigned by the coordinator"""
        # waiting for service to become live
        while not self.id_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for get_unique_id service...')
        # makign the request
        request = GetUniqueID.Request()
        request.requester_name = requester_name
        
        future = self.id_client.call_async(request) # sending the request (get a future in return which will be populated with the response)
        rclpy.spin_until_future_complete(self, future) # wait until populated

        if future.result() is not None:
            self.get_logger().info(f"ID assigned as: {future.result().id}")
            return future.result().id
        else:
            self.get_logger().error('ID Service call failed')
            return None

    def request_path(self, id):
        """Ask the coordinator for a new planner path without blocking callbacks."""
        if self.waiting_for_path_request:
            return True

        if not self.nav_path_client.service_is_ready():
            self.get_logger().info(f"robot{id} waiting for path service...")
            if not self.nav_path_client.wait_for_service(timeout_sec=0.1):
                return False

        request = GetNewFrontierPath.Request()
        request.requester_name = f"robot_{id}"
        request.requester_id = int(id)

        self.waiting_for_path_request = True
        future = self.nav_path_client.call_async(request)
        future.add_done_callback(functools.partial(self._path_response_callback, requester_id=id))
        return True

    def _path_response_callback(self, future, requester_id):
        """Handle planner service responses after the path topic callback lands."""
        self.waiting_for_path_request = False

        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f"path service call failed for robot{requester_id}: {exc}")
            return

        if result is None:
            self.get_logger().error(f"path service returned nothing for robot{requester_id}")
            return

        if result.success:
            self.get_logger().info(result.message)
            #  path message catching: service success can arrive before the one-shot PoseArray does.
            self.awaiting_nav_path_after_success = True
            self.next_path_retry_time = self.get_clock().now() + Duration(seconds=0.8)
            self.next_path_retry_wall_time = time.monotonic() + 0.8
            return

        self.get_logger().warn(result.message)
        self.next_path_retry_time = self.get_clock().now() + Duration(seconds=1.0)
        self.next_path_retry_wall_time = time.monotonic() + 1.0
        if self._fsm == fsm.RECOVERY:
            self._recovery_fsm = recovery_fsm.FINE
            self._fsm = fsm.ON
        elif self._fsm == fsm.WAITING_FOR_PATH:
            self.stop()

    def _wait_for_sim_ready(self, timeout_sec):
        """Wait until simulation clock and cmd_vel subscriber are ready."""
        self.get_logger().info('Waiting for simulation to be ready...')

        start_time = time.monotonic()
        clock_ready = not USE_SIM_TIME

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            now = self.get_clock().now()
            if time.monotonic() - start_time >= timeout_sec:
                self.get_logger().warn('Startup wait timeout reached. Continuing anyway.')
                return

            if USE_SIM_TIME and now.nanoseconds > 0:
                clock_ready = True

            cmd_ready = self._cmd_pub.get_subscription_count() > 0
            if clock_ready and not cmd_ready and time.monotonic() - start_time >= 3.0:
                self.get_logger().warn('Simulation clock ready, but cmd_vel subscriber was not discovered yet. Continuing startup.')
                return
            if clock_ready and cmd_ready:
                self.get_logger().info('Simulation ready. Node ready for activation service.')
                return
        
               

    def move(self, linear_vel, angular_vel):
        """Send a velocity command (linear vel in m/s, angular vel in rad/s)."""
        if self.done and self.stop_on_mission_complete:
            self.stop()
            return

        # Setting velocities.
        twist_msg = Twist()

        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        self._cmd_pub.publish(twist_msg)
        if (abs(linear_vel) > 0.0 or abs(angular_vel) > 0.0) and time.monotonic() >= self.next_motion_log_wall_time:
            self.get_logger().info(f"cmd_vel publishing: linear={linear_vel:.2f}, angular={angular_vel:.2f}")
            self.next_motion_log_wall_time = time.monotonic() + 1.0

    def stop(self):
        """Stop the robot."""
        twist_msg = Twist()
        self._cmd_pub.publish(twist_msg)

    def clamp(self, value, low, high):
        """Keep a numeric value inside a bounded interval."""
        return max(low, min(high, value))

    def scan_sector_min(self, msg, min_angle, max_angle):
        """Return the closest finite LiDAR range inside an angular sector."""
        closest = None
        for i, ray_length in enumerate(msg.ranges):
            if math.isnan(ray_length) or math.isinf(ray_length):
                continue
            if ray_length < msg.range_min or ray_length > msg.range_max:
                continue

            angle = msg.angle_min + i * msg.angle_increment
            if min_angle <= angle <= max_angle:
                closest = ray_length if closest is None else min(closest, ray_length)

        return msg.range_max if closest is None else closest

    def clearance_drive_command(self, default_linear):
        """Shape forward motion so LiDAR keeps a little breathing room around obstacles."""
        if self.latest_scan_msg is None:
            return default_linear, 0.0

        wall_now = time.monotonic()
        if wall_now - self.latest_scan_wall_time > CLEARANCE_SCAN_STALE_SEC:
            return default_linear, 0.0

        msg = self.latest_scan_msg
        front = self.scan_sector_min(msg, math.radians(-22), math.radians(22))
        front_left = self.scan_sector_min(msg, math.radians(22), math.radians(70))
        front_right = self.scan_sector_min(msg, math.radians(-70), math.radians(-22))
        left = self.scan_sector_min(msg, math.radians(70), math.radians(120))
        right = self.scan_sector_min(msg, math.radians(-120), math.radians(-70))

        linear = default_linear
        angular = 0.0

        if front < CLEARANCE_SOFT_RADIUS:
            left_space = min(front_left, left)
            right_space = min(front_right, right)
            turn_direction = 1.0 if left_space >= right_space else -1.0
            front_error = max(0.0, CLEARANCE_TARGET_RADIUS - front)
            angular += turn_direction * (CLEARANCE_FRONT_TURN + CLEARANCE_FRONT_GAIN * front_error / CLEARANCE_TARGET_RADIUS)

            if front <= CLEARANCE_HARD_STOP_RADIUS:
                linear = 0.0
            else:
                usable = (front - CLEARANCE_HARD_STOP_RADIUS) / (CLEARANCE_SOFT_RADIUS - CLEARANCE_HARD_STOP_RADIUS)
                linear = max(CLEARANCE_MIN_LINEAR, default_linear * self.clamp(usable, 0.15, 0.85))

        left_penalty = max(0.0, CLEARANCE_TARGET_RADIUS - min(left, front_left))
        right_penalty = max(0.0, CLEARANCE_TARGET_RADIUS - min(right, front_right))
        if left_penalty > 0.0 or right_penalty > 0.0:
            # clearance holding: right-side pressure turns left, left-side pressure turns right.
            angular += CLEARANCE_SIDE_GAIN * (right_penalty - left_penalty) / CLEARANCE_TARGET_RADIUS
            if front >= CLEARANCE_SOFT_RADIUS:
                linear = min(linear, default_linear * 0.82)

        angular = self.clamp(angular, -CLEARANCE_MAX_TURN, CLEARANCE_MAX_TURN)
        if abs(angular) > 0.42:
            linear = min(linear, default_linear * 0.65)

        if (abs(angular) > 0.05 or linear < default_linear * 0.95) and wall_now >= self.next_clearance_log_wall_time:
            self.get_logger().info(
                "clearance control: "
                f"front={front:.2f}m left={left:.2f}m right={right:.2f}m "
                f"cmd=({linear:.2f}, {angular:.2f})"
            )
            self.next_clearance_log_wall_time = wall_now + 1.0

        return linear, angular

    def begin_escape_recovery(self, reason, turn_direction=1.0, linear_velocity=ESCAPE_BACKUP_VELOCITY):
        """Start a small physical escape before asking for another planner path."""
        escape_word = "backing up" if linear_velocity < 0.0 else "nudging forward"
        self.get_logger().info(f"{reason}; {escape_word} before replanning")
        self.stop()
        self._local_fsm = local_fsm.IDLE
        self.busy = False
        self.nav_path = None
        self.recovery_path = None
        self._fsm = fsm.RECOVERY
        self._recovery_fsm = recovery_fsm.ESCAPING
        self.escape_stage = "backing"
        self.escape_turn_direction = 1.0 if turn_direction >= 0 else -1.0
        self.escape_linear_velocity = linear_velocity
        self.motion_watchdog_pose = None
        self.escape_done_wall_time = time.monotonic() + ESCAPE_BACKUP_TIME
        self.done_time_for_laser_wait = self.get_clock().now() + Duration(seconds=LASER_SLEEP_TIME_AFTER_INTERRUPT)

    def run_escape_recovery(self):
        """Run backup-turn recovery until the robot is ready for a new planner path."""
        wall_now = time.monotonic()
        if self.escape_stage == "backing":
            self.move(self.escape_linear_velocity, 0.0)
            if wall_now >= self.escape_done_wall_time:
                self.escape_stage = "turning"
                self.escape_done_wall_time = wall_now + ESCAPE_TURN_TIME
            return True

        if self.escape_stage == "turning":
            self.move(0.0, self.escape_turn_direction * ESCAPE_TURN_VELOCITY)
            if wall_now >= self.escape_done_wall_time:
                #  recovery finishing: now ask the global planner for a cleaner route.
                self.stop()
                self.escape_stage = None
                self._recovery_fsm = recovery_fsm.FINE
                self._fsm = fsm.WAITING_FOR_PATH
                self.next_path_retry_wall_time = wall_now
                self.request_path(self.id)
            return True

        self._recovery_fsm = recovery_fsm.FINE
        return False

    def watch_motion_progress(self):
        """Start escape recovery if commanded motion is not changing odometry."""
        if self._local_fsm != local_fsm.MOVING:
            self.motion_watchdog_pose = None
            self.motion_watchdog_wall_time = time.monotonic()
            return False

        x_odom, y_odom, _ = self.get_base_link_pose_in_odom(rclpy.time.Time())
        current_pose = (x_odom, y_odom)
        wall_now = time.monotonic()
        if self.motion_watchdog_pose is None:
            self.motion_watchdog_pose = current_pose
            self.motion_watchdog_wall_time = wall_now
            return False

        if self.euclidean_distance(current_pose, self.motion_watchdog_pose) >= STUCK_PROGRESS_EPS:
            self.motion_watchdog_pose = current_pose
            self.motion_watchdog_wall_time = wall_now
            return False

        if wall_now - self.motion_watchdog_wall_time >= STUCK_WATCHDOG_SEC:
            #  stuck noticing: commanded moving with no odom change means collision or traction loss.
            self.begin_escape_recovery(
                "Motion stall detected near obstacle",
                turn_direction=random.choice([-1.0, 1.0]),
            )
            return True

        return False

    def _turn_on_off_callback(self, req, resp):
        if not req.data: # if the request is false (ie. turn off)
            self._fsm = fsm.OFF
            self.stop()
            resp.success = True
            resp.message = "Robot stopped"
        else: # if the request is true (ie. turn on)
            if self._fsm == fsm.OFF:
                self._fsm = fsm.ON
                resp.success = True
                resp.message = "Robot activated"
            else:
                resp.success = False
                resp.message = "Robot already ON"
        
        return resp
    
    
    def start(self):
        """Wait for startup readiness and begin timer-driven control loop."""
        self._fsm = fsm.WAITING_FOR_PATH
        self._wait_for_sim_ready(STARTUP_TIMEOUT)
        #  clock decoupling: robot control should still tick when Gazebo /clock is late.
        self._control_timer = self.create_timer(1.0 / FREQUENCY, self._control_loop_callback, clock=Clock(clock_type=ClockType.STEADY_TIME))
        


    def shutdown_mapper(self):
        """shuts down the mapper"""
        # publishing that this node is NOT active!
        id_active_msg = Bool()
        id_active_msg.data = False
        self._id_active_pub.publish(id_active_msg)
        

    def check_TF_buffer_has_data(self, timestamp):
        """returns True if TF buffer has data, false if empty"""
        base_ready = self._tf_buffer.can_transform(
            self.odom_frame,         # The reference frame we are converting to (target)
            self.base_frame,         # The reference frame we are converting from (source)
            timestamp
        )
        laser_ready = self._tf_buffer.can_transform(
            self.odom_frame,
            self.laser_frame,
            timestamp
        )
        return base_ready and laser_ready

    def _nav_path_callback(self, msg):
        """callback function that updates the locally stored nav_path with the pose array published to this topic"""
        if self.done:
            #  mission holding: after final answer, old path rebroadcasts should not wake the robot.
            return

        # turning the pose array into an array of (x_odom, y_odom) points
        new_nav_path = []
        for pose in msg.poses:
            new_nav_path.append((pose.position.x, pose.position.y))

        # updating local nav_path with the new one
        if len(new_nav_path) == 0:
            # Empty path meaning: planner found where we already are, so ask again instead of sitting still.
            self.nav_path = None
            self.awaiting_nav_path_after_success = True
            self.next_path_retry_time = self.get_clock().now() + Duration(seconds=0.5)
            self.next_path_retry_wall_time = time.monotonic() + 0.5
            self.get_logger().warn("received empty nav path; requesting another waypoint soon")
            return

        if (
            self._fsm == fsm.EXECUTING_PATH
            and self.last_received_nav_path is not None
            and self.paths_match(new_nav_path, self.last_received_nav_path)
        ):
            #  duplicate path ignoring: coordinator rebroadcasts for reliability, but execution should keep going.
            return

        clipped_nav_path = self.clip_nav_path(new_nav_path)
        if len(clipped_nav_path) == 0:
            self.nav_path = None
            self.awaiting_nav_path_after_success = True
            self.next_path_retry_time = self.get_clock().now() + Duration(seconds=0.5)
            self.next_path_retry_wall_time = time.monotonic() + 0.5
            self.get_logger().warn("received nav path clipped to nothing; requesting another waypoint soon")
            return

        self.nav_path = clipped_nav_path
        self.last_received_nav_path = list(new_nav_path)
        self.awaiting_nav_path_after_success = False
        self.busy = False
        self.done = False
        self._local_fsm = local_fsm.IDLE
        self.last_nav_path_timestamp = msg.header.stamp
        self.get_logger().info(f"received nav path with {len(self.nav_path)} waypoint(s)")
        if self._fsm != fsm.RECOVERY:
            self._fsm = fsm.EXECUTING_PATH
        elif self._recovery_fsm == recovery_fsm.WAITING_FOR_PATH:
            self.get_recovery_path()

    def paths_match(self, first_path, second_path, tolerance=0.03):
        """Return true when two odom paths are effectively the same rebroadcast."""
        if len(first_path) != len(second_path):
            return False

        for first, second in zip(first_path, second_path):
            if self.euclidean_distance(first, second) > tolerance:
                return False
        return True

    def clip_nav_path(self, nav_path):
        """Return the path from the waypoint nearest the robot onward."""
        if len(nav_path) == 0:
            return []

        try:
            x, y, _theta = self.get_base_link_pose_in_odom(rclpy.time.Time())
        except Exception as exc:
            self.get_logger().warn(f"could not clip nav path without robot pose: {exc}")
            return nav_path

        robot_pos = (x, y)
        best_index = 0
        best_distance = self.euclidean_distance(nav_path[0], robot_pos)
        for idx, point in enumerate(nav_path):
            distance = self.euclidean_distance(point, robot_pos)
            if distance <= best_distance:
                best_distance = distance
                best_index = idx

        return nav_path[best_index:]



    def _laser_callback(self, msg):
        """Processing of laser message."""
        self.latest_scan_msg = msg
        self.latest_scan_wall_time = time.monotonic()

        # Access to the index of the measurement in front of the robot.
        # LaserScan message https://docs.ros2.org/foxy/api/sensor_msgs/msg/LaserScan.html
        # NOTE: index 0 corresponds to min_angle, 
        #       index 1 corresponds to min_angle + angle_inc
        #       index 2 corresponds to min_angle + angle_inc * 2
        #       ...
        # the lidar scanner is positioned such that angle=0 corresponds with the x-axis (forward direction)
        timestamp = rclpy.time.Time.from_msg(msg.header.stamp)
        if not self.check_TF_buffer_has_data(timestamp): # if we don't have any tranform data we can't do any mapping, so ignore laser data
            # we have to do this becuase I was running into an issue where it would crash because it couldn't find the transfer frame  
            # in the buffer because update_SLAM_map() was called before the buffer got loaded (presumably because laser data came in first)
            return

        if not self.SLAM_map_initialized: # if the map hasn't been initialized
            self.max_sensor_range = msg.range_max
            self.init_SLAM_map(self.max_sensor_range, self.SLAM_MAP_RESOLUTION_SCALAR) # call initialization function
            self.SLAM_map_initialized = True # mark map initialized

        # we have received a scan! decrement scan_downsampling_count, if it gets to 0 we reset to downsampling n and process the scan into the SLAM map
        self.scan_downsampling_count -=1
        if self.scan_downsampling_count > 0:
            return
        else:
            self.scan_downsampling_count = self.SCAN_DOWNSAMPLING # and continue processing the scan
        
        # iterating through every angle received in msg.
        for i in range(0, len(msg.ranges)):
            angle = msg.angle_min + i*msg.angle_increment
            self.update_SLAM_map(timestamp, angle, msg.ranges[i], msg.range_max) # calling our update slam map function for each laser ray
        
        # SLAM map has been updated, now we check if we are too close to an obstacle and need to find a new path
        if (
            self.get_clock().now() > self.done_time_for_laser_wait
            and not self.waiting_for_path_request
            and self._recovery_fsm not in (recovery_fsm.EXECUTING_PATH, recovery_fsm.ESCAPING)
        ):
            for i, ray_length in enumerate(msg.ranges):
                if math.isnan(ray_length) or math.isinf(ray_length):
                    continue
                angle = msg.angle_min + i * msg.angle_increment
                forward_hit = abs(angle) <= FORWARD_PROTECTION_ANGLE_RAD and ray_length <= self.PROTECTION_RADIUS
                emergency_hit = ray_length <= EMERGENCY_PROTECTION_RADIUS
                if forward_hit or emergency_hit:
                    escape_linear_velocity = ESCAPE_BACKUP_VELOCITY
                    if emergency_hit and not forward_hit and abs(angle) > math.pi / 2:
                        #  rear bump handling: backing up would press harder into the thing behind us.
                        escape_linear_velocity = ESCAPE_FORWARD_VELOCITY
                    turn_direction = -1.0 if angle > 0 else 1.0
                    self.begin_escape_recovery(
                        "Found a point within protection radius",
                        turn_direction=turn_direction,
                        linear_velocity=escape_linear_velocity,
                    )
                    break
        # now that our slam map is updated, we publish it (publish includes a header creation)
        self.publish_SLAM_map(timestamp)
    
    def init_SLAM_map(self, max_sensor_range, resolution_scalar):
        """initilizes the slam map to some scalar multiple of the maximum sensor range"""
        # setting size, we will make map square
        self.SLAM_MAP_SIZE_X = int(resolution_scalar*2*max_sensor_range) # we add in the 2 because the sensor range is a ray, and the robot can see max range to its right and left
        self.SLAM_MAP_SIZE_Y = int(resolution_scalar*2*max_sensor_range)
        # the sensor range is in odom units already, so a sensor range of 20 at scale of 1 would result in a 40x40 map with the robot in the center
        # with a resolution_scale of 1 we'd each cell would have to be equal to 1m, the sensor range is already in m
        # resolution must be [m/cell], there's scalar*2*max_sensor_range cells, but the actual distance is 2*max_sensor_range --> m/cells  = 1/scalar
        self.SLAM_map_res_m_per_cell = 1/resolution_scalar

        # set origin of slam map to the middle of the grid, meaning odom origin is in 
        self.SLAM_map_origin_X = -self.SLAM_MAP_SIZE_X*self.SLAM_map_res_m_per_cell / 2
        self.SLAM_map_origin_Y = -self.SLAM_MAP_SIZE_Y*self.SLAM_map_res_m_per_cell / 2

        self.SLAM_map_info = self.create_SLAM_map_info() # this holds information about the occupancy grid map we generate: .width and .height
        if self.PROBABILISTIC_MAPPING:
            self.SLAM_map = np.zeros((self.SLAM_MAP_SIZE_Y, self.SLAM_MAP_SIZE_X), dtype=np.float32)
        else:
            self.SLAM_map = np.full((self.SLAM_MAP_SIZE_Y, self.SLAM_MAP_SIZE_X), -1, dtype=np.int8)
        

    def expand_SLAM_map(self, border_width:int):
        """Expands the SLAM_map by border_width cells, keeping the center in the center"""
        if self.SLAM_map is None: # if map hasn't been intiialized yet, do nothing
            return
        # getting new map size
        new_size_X = int(self.SLAM_MAP_SIZE_X + 2*border_width)
        new_size_Y = int(self.SLAM_MAP_SIZE_Y + 2*border_width)
        
        # empty new map
        if self.PROBABILISTIC_MAPPING:
            new_map = np.zeros((new_size_Y, new_size_X), dtype=np.float32)
        else:
            new_map = np.full((new_size_Y, new_size_X), -1, dtype=np.int8)

        # copy over data from old map
        for x in range(self.SLAM_MAP_SIZE_X):
            for y in range(self.SLAM_MAP_SIZE_Y):
                new_map[y+border_width][x+border_width] = self.SLAM_map[y][x]
        # updating class variables with new values
        self.SLAM_MAP_SIZE_X = new_size_X
        self.SLAM_MAP_SIZE_Y = new_size_Y
        self.SLAM_map = new_map
        self.SLAM_map_origin_X -= border_width * self.SLAM_map_res_m_per_cell
        self.SLAM_map_origin_Y -= border_width * self.SLAM_map_res_m_per_cell

    def create_SLAM_map_info(self):
        """this function creates a message header for the slam_map using the current time as timestamp"""
        # https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/MapMetaData.html
        
        slam_map_info = MapMetaData() # creating the metadata object
        slam_map_info.map_load_time = self.get_clock().now().to_msg() # using the current time as timestamp
        
        # set map resolution, width, height
        slam_map_info.resolution = self.SLAM_map_res_m_per_cell  # [m/cell]
        slam_map_info.width = self.SLAM_MAP_SIZE_X
        slam_map_info.height = self.SLAM_MAP_SIZE_Y

        # set the origin point as bottom left
        slam_map_info.origin = Pose() # using a pose object https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Pose.html
        slam_map_info.origin.position = Point(x=self.SLAM_map_origin_X, y=self.SLAM_map_origin_Y, z=0.0) # point position
        slam_map_info.origin.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0) # quaterinion orientation

        return slam_map_info # retruning the mapmetadata object


    def update_SLAM_map(self, timestamp, laser_angle, ray_length, range_max): 
        """This function updates the slam map based on a laser ray, it takes in the angle of the ray, the range data from the ray, and the max range of the sensor"""

        # now we get the start and end point of the ray in space
        endpoint_clear = False # if the end point of the laser ray terminates in a wall this is false, if it just terminates at max distance, 
        if math.isinf(ray_length): # if the range data is infinity, we don't detect anything up to 20 m away
            endpoint_clear = True
            ray_length = range_max
        
        # now we find the endpoint of the ray
        ray_endpoint_x = ray_length*math.cos(laser_angle)
        ray_endpoint_y = ray_length*math.sin(laser_angle)

        ray_endpoint_odom_x, ray_endpoint_odom_y = self.get_laser_point_in_odom(timestamp, ray_endpoint_x, ray_endpoint_y)
        ray_startpoint_odom_x,ray_startpoint_odom_y = self.get_laser_point_in_odom(timestamp, 0,0)

        # converting the ray start and end points into slam_map grid cell coordinates
        ray_startpoint_grid_x, ray_startpoint_grid_y = self.odom_to_cell((ray_startpoint_odom_x,ray_startpoint_odom_y))
        ray_endpoint_grid_x, ray_endpoint_grid_y = self.odom_to_cell((ray_endpoint_odom_x,ray_endpoint_odom_y)) 

        # now we check if the endpoint (map coordiante) is within the SLAM_map bounds, if not we expand the map
        if (ray_endpoint_grid_x < 0 or self.SLAM_MAP_SIZE_X <= ray_endpoint_grid_x or 
            ray_endpoint_grid_y < 0 or self.SLAM_MAP_SIZE_Y <= ray_endpoint_grid_y):
            # we will expand the map to be double it's current size (so border witdh is floor(map_side/2))
            self.expand_SLAM_map(self.SLAM_MAP_SIZE_X//2) # TODO: uncomment later
            return

        # if the endpoint is clear, then the all cells the ray passes through are marked as free space
        # if the endpoint is not clear, then all cells the ray passes through are markes as free space and the endpoint cell is marked as occupied

        # we will use the bresenham algorithm to enocde free space along the ray - https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm#All_cases
        x0, y0 = ray_startpoint_grid_x, ray_startpoint_grid_y
        x1, y1 = ray_endpoint_grid_x, ray_endpoint_grid_y

        dx = abs(x1-x0)
        dy = abs(y1-y0)

        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1

        err = dx - dy
        step = 0
        while True:
            step+=1
            if x0 == x1 and y0 == y1: # if we're at endpoint
                break  

            # mark traversed cell as free
            if self.PROBABILISTIC_MAPPING:
                self.SLAM_map[y0][x0] += self.get_log_odds_update(step, endpoint_clear, is_endpoint=False)
            else:
                self.SLAM_map[y0][x0] = 0

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy  

        # marking the end point as either clear or occupied
        if self.PROBABILISTIC_MAPPING:
            self.SLAM_map[y0][x0] += self.get_log_odds_update(step, endpoint_clear, is_endpoint=True)
        else:
            if endpoint_clear:
                self.SLAM_map[ray_endpoint_grid_y][ray_endpoint_grid_x] = 0 # mark endpoint as free
            else:
                self.SLAM_map[ray_endpoint_grid_y][ray_endpoint_grid_x] = 100 # if the ray end isn't clear we mark end as occupied
        
    def get_log_odds_update(self, step, endpoint_clear, is_endpoint):
        """Function for probabalistic mapping, gets returns the log odd of the sensor measurementat at given point on the grid"""
        confidence = 1.0 / (step * self.SLAM_map_res_m_per_cell + 1.0)
        if is_endpoint and not endpoint_clear:
            return math.log(0.9 / 0.1) * confidence  # occupied
        else:
            return math.log(0.3 / 0.7) * confidence  # free
        
    def log_odds_to_occupancy(self, log_odds_map):
        """updates returns a occupancy grid from -1 to 100 values given log odds grid"""
        clamped = np.clip(log_odds_map, -10, 10)  # overflow prevention
        probability = 1.0 - 1.0 / (1.0 + np.exp(clamped))
        occupancy = (probability * 100).astype(np.int8)
        occupancy[np.abs(log_odds_map) < 0.01] = -1
        return occupancy

    def publish_SLAM_map(self, timestamp):
        """This function publishes the current slam map """
        msg = OccupancyGrid() #creating the occupancyGrid message --the data from the occupancy grid: https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html

        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg() # timestamp assigned to the slam map

        self.SLAM_map_info = self.create_SLAM_map_info() # creating the slam map info we wwant
        msg.info = self.SLAM_map_info #  assigning info

        if self.PROBABILISTIC_MAPPING:
            flattened_SLAM_map = self.log_odds_to_occupancy(self.SLAM_map).flatten().tolist()
        else:
            flattened_SLAM_map = self.SLAM_map.flatten().tolist() # flattenign the slam map, msg.data requires a flat list of int8s
        msg.data = flattened_SLAM_map # assigning data

        self._SLAM_map_pub.publish(msg) # publishing the message to our SLAM map topic

        # Afer pulishing the slam map, we know that it has data in it, so mark flag booelan true
        if not self.SLAM_map_populated:
            self.SLAM_map_populated = True
            #  planner starting: the control timer owns requests, so startup mapping cannot trap us.
            self.next_path_retry_time = self.get_clock().now()
            self.next_path_retry_wall_time = time.monotonic()


    def get_recovery_path(self):
        """Build a local recovery path back onto the newest planner path."""
        self._recovery_fsm = recovery_fsm.WAITING_FOR_PATH
        if self.nav_path is None or len(self.nav_path) == 0:
            return

        if self.PROBABILISTIC_MAPPING:
            slam_occupancy = self.log_odds_to_occupancy(self.SLAM_map)
        else:
            slam_occupancy = self.SLAM_map

        smoothed_SLAM_map = self.gaussianSmoothing(slam_occupancy, SMOOTHING_KERNEL_SIZE, SMOOTHING_SIGMA)
        inflated_SLAM_map = self.obstacle_inflation(slam_occupancy, PROTECTION_RADIUS, self.SLAM_map_res_m_per_cell)

        obstacle_avoidance_search_map = np.zeros_like(slam_occupancy, dtype=np.float32)
        obstacle_avoidance_search_map[slam_occupancy == -1] = -1
        obstacle_avoidance_search_map[inflated_SLAM_map == 100] = 100

        free_cells = (slam_occupancy != -1) & (inflated_SLAM_map != 100)
        obstacle_avoidance_search_map[free_cells] = smoothed_SLAM_map[free_cells]

        x_odom, y_odom, _ = self.get_base_link_pose_in_odom(rclpy.time.Time())
        robot_cell = self.odom_to_cell((x_odom, y_odom))
        robot_cell = self.get_nearest_free_cell(robot_cell[0], robot_cell[1], obstacle_avoidance_search_map)

        #  Recovery choosing: meet the planned route at the nearest valid point.
        nav_path_reverse = [self.odom_to_cell(pt) for pt in self.nav_path[::-1]]
        valid_points = [
            pt for pt in nav_path_reverse
            if 0 <= pt[0] < self.SLAM_MAP_SIZE_X
            and 0 <= pt[1] < self.SLAM_MAP_SIZE_Y
            and 0 <= obstacle_avoidance_search_map[pt[1]][pt[0]] < MAP_CLEAR_THRESHOLD
        ]
        if len(valid_points) == 0:
            self.get_logger().warn("No valid recovery join point on the current nav path")
            self.recovery_path = []
            self._recovery_fsm = recovery_fsm.EXECUTING_PATH
            return

        best_point = min(valid_points, key=lambda pt: self.euclidean_distance(pt, robot_cell))
        path_cells = self.a_star_path(robot_cell, best_point, obstacle_avoidance_search_map, self.SLAM_MAP_SIZE_X, self.SLAM_MAP_SIZE_Y)

        self.recovery_path = []
        if path_cells is not None:
            self.recovery_path = [self.cell_to_world(point) for point in path_cells]
        self._recovery_fsm = recovery_fsm.EXECUTING_PATH

    def a_star_path(self, start_point, goal_point, obstacle_avoidance_search_map, map_width, map_height):
        """Run A* and return cell coordinates from start to goal."""
        seen_cells = np.zeros((map_height, map_width), dtype=bool)
        priority_queue = []
        counter = 0
        seen_cells[start_point[1]][start_point[0]] = True

        root_node = TreeNode("root")
        root_node.x = start_point[0]
        root_node.y = start_point[1]
        root_node.cost = 0

        goal_node = None
        heapq.heappush(priority_queue, (self.euclidean_distance(start_point, goal_point), counter, root_node))

        while len(priority_queue) != 0:
            _, _, nextup = heapq.heappop(priority_queue)
            if nextup.y == goal_point[1] and nextup.x == goal_point[0]:
                goal_node = nextup
                break

            for neighbor in NEIGHBOR_LIST:
                x_n, y_n = neighbor[0] + nextup.x, neighbor[1] + nextup.y
                if (0 <= x_n < map_width and 0 <= y_n < map_height) and 0 <= obstacle_avoidance_search_map[y_n][x_n] <= MAP_CLEAR_THRESHOLD:
                    if not seen_cells[y_n][x_n]:
                        neighbor_node = TreeNode("child")
                        neighbor_node.parent = nextup
                        neighbor_node.x = x_n
                        neighbor_node.y = y_n
                        neighbor_node.cost = nextup.cost + self.euclidean_distance((nextup.x, nextup.y), (x_n, y_n))

                        weight = neighbor_node.cost + obstacle_avoidance_search_map[y_n][x_n] + self.euclidean_distance((x_n, y_n), goal_point)
                        heapq.heappush(priority_queue, (weight, counter, neighbor_node))
                        counter += 1
                        seen_cells[y_n][x_n] = True

        if goal_node is None:
            self.get_logger().warn("A* search not able to find a reachable recovery goal")
            return None

        a_star_node_path = [goal_node]
        while a_star_node_path[0].name != "root":
            a_star_node_path.insert(0, a_star_node_path[0].parent)
        return [(n.x, n.y) for n in a_star_node_path]

    def gaussianSmoothing(self, to_smooth_array, kernel_size, sigma):
        """Apply Gaussian smoothing to an occupancy grid."""
        if to_smooth_array is None:
            return None

        height = to_smooth_array.shape[0]
        width = to_smooth_array.shape[1]

        kernel = []
        for y in range(-kernel_size//2, kernel_size//2 + 1):
            row = []
            for x in range(-kernel_size//2, kernel_size//2 + 1):
                row.append(1/(2*math.pi*sigma**2) * math.exp(-1*(x**2 + y**2)/(2*sigma**2)))
            kernel.append(row)

        kernel = np.array(kernel)
        kernel = kernel / np.sum(kernel)

        smoothed_map = np.zeros((height, width), dtype=np.float32)
        for y in range(0, height):
            for x in range(0, width):
                neighborhood = []
                for y_n in range(-kernel_size//2, kernel_size//2 + 1):
                    neighborhood_row = []
                    for x_n in range(-kernel_size//2, kernel_size//2 + 1):
                        if 0 <= y + y_n < height and 0 <= x + x_n < width:
                            if to_smooth_array[y + y_n][x + x_n] >= 0:
                                neighborhood_row.append(to_smooth_array[y + y_n][x + x_n])
                            else:
                                neighborhood_row.append(0)
                        else:
                            neighborhood_row.append(0)
                    neighborhood.append(neighborhood_row)
                smoothed_map[y, x] = np.sum(np.array(neighborhood, dtype=np.float32) * kernel)

        max_val = np.max(smoothed_map)
        if max_val != 0:
            scaled = (smoothed_map / max_val) * 100
        else:
            scaled = smoothed_map

        scaled[smoothed_map == 0] = 0
        smoothed_map = scaled.astype(np.int8)
        for y in range(0, height):
            for x in range(0, width):
                if to_smooth_array[y][x] == 100:
                    smoothed_map[y][x] = 100
                if to_smooth_array[y][x] == -1:
                    smoothed_map[y][x] = -1

        return smoothed_map

    def obstacle_inflation(self, to_inflate_map, protection_radius, resolution):
        """Inflate occupied cells by the robot protection radius."""
        if to_inflate_map is None:
            return None

        inflated = np.copy(to_inflate_map)
        height, width = to_inflate_map.shape
        radius_cells = int(protection_radius//resolution + 1)
        obstacle_cells = np.argwhere(to_inflate_map >= MAP_OCCUPIED_THRESHOLD)
        for y, x in obstacle_cells:
            for dy in range(-radius_cells, radius_cells + 1):
                for dx in range(-radius_cells, radius_cells + 1):
                    if dx*dx + dy*dy <= radius_cells**2:
                        ny = y + dy
                        nx = x + dx
                        if 0 <= ny < height and 0 <= nx < width:
                            inflated[ny][nx] = 100

        return inflated

    def euclidean_distance(self, p1, p2):
        """returns euclidean distance between 2 points (x,y) tuple"""
        return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)




    def get_nearest_free_cell(self, x, y, search_map):
        """Returns nearest free cell to inputted point, searching outward in a spiral"""
        for radius in range(0, search_map.shape[0]):
            for dx in range(-radius, radius+1):
                for dy in range(-radius, radius+1):
                    if abs(dx) != radius and abs(dy) != radius:
                        continue
                    nx, ny = x + dx, y + dy
                    if (0 <= nx < self.SLAM_MAP_SIZE_X and
                        0 <= ny < self.SLAM_MAP_SIZE_Y and
                        0 <= search_map[ny][nx] <= MAP_CLEAR_THRESHOLD):
                        return nx, ny
        return x, y

    def cell_to_world(self, cell_point):
        """converts pixel/cell point values to world points, returns in (x,y) tuple form"""
        origin_x = self.SLAM_map_info.origin.position.x
        origin_y = self.SLAM_map_info.origin.position.y
        res = self.SLAM_map_info.resolution

        x_world = origin_x + (cell_point[0])*res #we add the 0.5 to make the conversion map to the center of the cell, not the upper right
        y_world = origin_y + (cell_point[1])*res

        return (x_world, y_world)
    

    def odom_to_cell(self, odom_point):
        """converts points in odom to pixel/cell values"""
        origin_x = self.SLAM_map_info.origin.position.x
        origin_y = self.SLAM_map_info.origin.position.y
        res = self.SLAM_map_info.resolution

        cell_point_x = round((odom_point[0] - origin_x)/res)  
        cell_point_y = round((odom_point[1] - origin_y)/res)
        
        return (cell_point_x, cell_point_y)


    def get_base_link_pose_in_odom(self, timestamp):
        """This function returns (x,y,theta) position and rotation of the robot (base_link) in the odom reference frame"""
        if self.latest_odom_msg is not None:
            pose = self.latest_odom_msg.pose.pose
            robot_angle_quat = pose.orientation
            robot_angle = math.atan2(
                2 * (robot_angle_quat.w*robot_angle_quat.z + robot_angle_quat.x*robot_angle_quat.y),
                1 - 2*(robot_angle_quat.y*robot_angle_quat.y + robot_angle_quat.z*robot_angle_quat.z)
            )
            return (pose.position.x, pose.position.y, robot_angle)

        transform = self._tf_buffer.lookup_transform(
            self.odom_frame,       # The reference frame we are converting to (target)
            self.base_frame,       # The reference frame we are converting from (source)
            timestamp
        )
                
        # current point of robot
        x_pos_odom = transform.transform.translation.x # x coordinate of base link in odom reference frame
        y_pos_odom = transform.transform.translation.y # y coordinate of base link in odom reference frame

        # current angle of robot
        robot_angle_quat = transform.transform.rotation # curent angle as quaternion
        robot_angle = math.atan2(
            2 * (robot_angle_quat.w*robot_angle_quat.z + robot_angle_quat.x*robot_angle_quat.y),
            1 - 2*(robot_angle_quat.y*robot_angle_quat.y + robot_angle_quat.z*robot_angle_quat.z)
        ) # converting to regular angle

        # current 
        return (x_pos_odom, y_pos_odom, robot_angle)


    def publish_pose(self, timestamp):
        """This function publishes this robots pose (relative to odom reference frame) to self._pose_pub"""
        if self.latest_odom_msg is not None:
            msg = PoseStamped()
            msg.header.stamp = timestamp.to_msg()
            msg.header.frame_id = self.map_frame
            msg.pose = self.latest_odom_msg.pose.pose
            self._pose_pub.publish(msg)
            return True

        # first getting the pose of the robot relative to odom
        try:
            transform = self._tf_buffer.lookup_transform(
                self.odom_frame,       # The reference frame we are converting to (target)
                self.base_frame,       # The reference frame we are converting from (source)
                rclpy.time.Time() # this time value is like 0/null so it returns latest availiable transform
            )
        except Exception as exc:
            if self.pose_warning_count % FREQUENCY == 0:
                self.get_logger().warn(f"Waiting for robot pose before publishing pose: {exc}")
            self.pose_warning_count += 1
            return False
                
        # current point of robot
        x_pos_odom = transform.transform.translation.x # x coordinate of base link in odom reference frame
        y_pos_odom = transform.transform.translation.y # y coordinate of base link in odom reference frame

        # current angle of robot
        robot_angle_quat = transform.transform.rotation # curent angle as quaternion

        # creating the message
        msg = PoseStamped()
        # filling in the data
        msg.header.stamp = timestamp.to_msg()
        msg.header.frame_id = self.map_frame

        msg.pose.position.x = x_pos_odom
        msg.pose.position.y = y_pos_odom
        msg.pose.orientation = robot_angle_quat
        #publishing the message
        self._pose_pub.publish(msg)
        return True


    def get_laser_point_in_odom(self, timestamp, x, y):
        """This function returns (x,y,theta) position and rotation of the robot (base_link) in the odom reference frame"""
        transform = self._tf_buffer.lookup_transform(
            self.odom_frame,       # The reference frame we are converting to (target)
            self.laser_frame,      # The reference frame we are converting from (source)
            timestamp
        )
                
        # current point of robot
        x_translation_offset = transform.transform.translation.x # x coordinate of base link in odom reference frame
        y_translation_offset = transform.transform.translation.y # y coordinate of base link in odom reference frame

        # current angle of robot
        angle_quat = transform.transform.rotation # curent angle as quaternion
        angle = math.atan2(
            2 * (angle_quat.w*angle_quat.z + angle_quat.x*angle_quat.y),
            1 - 2*(angle_quat.y*angle_quat.y + angle_quat.z*angle_quat.z)
        ) # converting to regular angle

        x_odom = math.cos(angle)*x - math.sin(angle)*y + x_translation_offset
        y_odom = math.sin(angle)*x + math.cos(angle)*y + y_translation_offset

        # current 
        return (x_odom, y_odom)



    def _control_loop_callback(self): # will be called every self.delta_t seconds 

        if self._fsm == fsm.OFF: # if robot is off, do nothing
            return

        # publish the robot's position
        pose_published = self.publish_pose(self.get_clock().now())

        if (
            pose_published
            and self.SLAM_map_populated
            and self.nav_path is None
            and not self.waiting_for_path_request
            and time.monotonic() >= self.next_path_retry_wall_time
        ):
            self._fsm = fsm.WAITING_FOR_PATH
            self.request_path(self.id)

        if self._fsm==fsm.WAITING_FOR_PATH : # if robot is waiting for path, we do nothing
            if (
                self.awaiting_nav_path_after_success
                and not self.waiting_for_path_request
                and time.monotonic() >= self.next_path_retry_wall_time
            ):
                #  retrying path delivery: the service answered, but the PoseArray did not land.
                self.get_logger().warn("nav path message not received after service success; retrying path request")
                self.next_path_retry_time = self.get_clock().now() + Duration(seconds=1.0)
                self.next_path_retry_wall_time = time.monotonic() + 1.0
                self.request_path(self.id)
            return
        
        if self._fsm == fsm.RECOVERY and  self._recovery_fsm == recovery_fsm.WAITING_FOR_PATH:
            return
        
        if self.done:
            self.stop() # stop robot
            self._fsm = fsm.OFF # turn off robot
            return

        if self._fsm == fsm.RECOVERY and self._recovery_fsm == recovery_fsm.ESCAPING:
            if self.run_escape_recovery():
                return
        

        if not self.busy and (self.nav_path is not None or self._fsm == fsm.RECOVERY): # if we're not in the middle of an action
            if self._fsm == fsm.RECOVERY and (self.recovery_path is None or len(self.recovery_path) == 0):
                self.stop()
                self._fsm = fsm.WAITING_FOR_PATH
                self._recovery_fsm = recovery_fsm.FINE
                self.recovery_path = None
                self._local_fsm = local_fsm.IDLE
                self.request_path(self.id)
                return

            if self.nav_path is not None and len(self.nav_path) == 0: # if we are out of points in the desired path
                self.nav_path = None
                self._fsm=fsm.WAITING_FOR_PATH
                self.stop()
                self._local_fsm = local_fsm.IDLE
                self.get_logger().info("waypoint reached; requesting next planner path")
                self.request_path(self.id)
                return


            if self._fsm == fsm.RECOVERY:
                self.next_point_world = self.recovery_path.pop(0)
            else:
                # path Following: take the next planner point.
                self.next_point_world = self.nav_path.pop(0)


            current_point_x, current_point_y, robot_angle = self.get_base_link_pose_in_odom(rclpy.time.Time())

            # getting the angle and distance to the first point in the list
            # we now need the amount of rotation to face the next point and the distance traveled to the next point
            desired_angle = math.atan2(np.round(self.next_point_world[1]-current_point_y, decimals=2), np.round(self.next_point_world[0]-current_point_x, decimals=2))
            rotation = desired_angle - robot_angle # our desired angle
            rotation_to_next = (rotation+math.pi) % (2*math.pi) - math.pi # normalizing the angle to between pi and -pi so that we turn the least we have to
            distance_to_next = math.sqrt((self.next_point_world[0]-current_point_x)**2 + (self.next_point_world[1]-current_point_y)**2)
            if distance_to_next < 0.05:
                # waypoint skipping: tiny local steps do not show up in Gazebo and can stall the demo visually.
                self.busy = False
                self._local_fsm = local_fsm.IDLE
                return

            self.get_logger().info(
                f"driving to waypoint ({self.next_point_world[0]:.2f}, {self.next_point_world[1]:.2f}), "
                f"distance={distance_to_next:.2f} m"
            )

            #figuring out the time the rotation will take
            sleep_time = abs(rotation_to_next/self.ANGULAR_VELOCITY)
            duration = Duration(seconds=sleep_time)
            self.rotation_done_time = self.get_clock().now() + duration # updating the rotation done time
            self.rotation_done_wall_time = time.monotonic() + sleep_time
            # figuring out the time the translation will take
            sleep_time = distance_to_next/self.LINEAR_VELOCITY
            duration = Duration(seconds=sleep_time)
            self.translation_done_time = self.rotation_done_time + duration # updating the translation done time (will be executed after the rotation so we must account for that)
            self.translation_done_wall_time = self.rotation_done_wall_time + sleep_time

            # local action setting
            self._local_fsm = local_fsm.ROTATING
            #store the angular velocity for the rotation
            self.omega = self.ANGULAR_VELOCITY 
            if rotation_to_next < 0:
                self.omega = -self.omega
            self.busy = True
        
        if self.busy: # if we are in the middle of executing an action
            #if we are busy
            if self._local_fsm == local_fsm.ROTATING: # if we are in rotation state
                self.move(0.0, self.omega) # send the rotation command
                if time.monotonic() > self.rotation_done_wall_time: # if we are done, update the state to moving (move after a rotation)
                    self._local_fsm=local_fsm.MOVING

            if self._local_fsm == local_fsm.MOVING: # if we are in the movement state
                if self.watch_motion_progress():
                    return
                linear_cmd, angular_cmd = self.clearance_drive_command(self.LINEAR_VELOCITY)
                self.move(linear_cmd, angular_cmd) # send the move command with LiDAR clearance shaping
                if time.monotonic() > self.translation_done_wall_time: # if the movement action is done, we are done with this action and we set busy to false
                    self.busy = False
                    self.motion_watchdog_pose = None
                    return

            



def main(args=None):
    """Main function."""

    # 1st. initialization of node.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    # Initialization of the class for the world mapper.
    world_mapper = WorldMapper()

    # navigates and maps the environment.
    try:
        world_mapper.start()
        rclpy.spin(world_mapper)
    except KeyboardInterrupt:
        world_mapper.shutdown_mapper()
        world_mapper.get_logger().error("ROS node interrupted.")
    finally:
        if rclpy.ok():
            world_mapper.stop()
        world_mapper.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    """Run the main function."""
    main()

    


# potentially useful code from past assignments....
    # def rotate_angle(self, angle):
    #     """This function rotates the robot an inputted angle (in radians) at self.angular_velocity"""
    #     omega = self.ANGULAR_VELOCITY 
    #     if angle < 0:
    #         omega = -omega
    #     elif angle==0: #dont' want a divide by zero error
    #         return self.get_clock().now()

    #     sleep_time = abs(angle/self.ANGULAR_VELOCITY)
    #     duration = Duration(seconds=sleep_time)
    #     self.move(0.0, omega) #turn
    #     self.busy = True
    #     return  self.get_clock().now() + duration


    # def move_distance(self, distance):
    #     """This function moves the robot for the inputted distance (in meters) at self.linear_velocity.
    #     The relevant values are set to the motors, action_done time is updated"""
    #     sleep_time = distance/self.LINEAR_VELOCITY
    #     duration = Duration(seconds=sleep_time)
    #     self.move(self.LINEAR_VELOCITY, 0.0)
    #     self.busy = True
    #     return  self.get_clock().now() + duration
