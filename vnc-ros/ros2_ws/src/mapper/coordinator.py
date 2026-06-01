#!/usr/bin/env python
#The line above is important so that this file is interpreted with Python when running it.

# Multi-robot planning and coordination node.

# Import of python modules.
import math # use of pi.
import random # use for generating a random real number
from enum import Enum
import time
import numpy as np # for map grid representations and operations
from anytree import Node as TreeNode # for search algorithms (so important to import as Tree node because we have a Node class from ros2 already
import heapq # for A*
import functools # for partial funciton calling
from collections import deque

# import of relevant libraries.
import rclpy # module for ROS APIs
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.clock import Clock, ClockType
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid # message type for occupancyGrid
from  nav_msgs.msg import MapMetaData # for the slam_map msg.info
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Point, Quaternion, PointStamped, Twist # for the ifnromation stored in slam_map msg.info
from std_msgs.msg import Bool, Int32 # for id_active publisher


# importing custom services
from mapper_interfaces.srv import GetUniqueID
from mapper_interfaces.srv import GetNewFrontierPath

## Constants ##
NEIGHBOR_LIST = [  # list of relative neighbors to a node
        (-1,1),  (0,1),  (1,1),
        (-1,0),          (1,0),
        (-1,-1), (0,-1), (1,-1)  
        ]     
# NEIGHBOR_LIST = [
#                   (0,1),
#         (-1,0),          (1,0),
#                 (0,-1),  
#         ] 


PROTECTION_RADIUS = 0.3 # [m]
HEURISTIC_KEEP_OUT_RADIUS = 0.9 # [m]
HEURISTIC_NEAR_ROBOT_IGNORE_RADIUS = 1.25 # [m]
HEURISTIC_FRONTIER_BIAS_WEIGHT = 0.18
GOAL_APPROACH_RADIUS = 0.25 # [m]
GOAL_APPROACH_PATH_STEPS = 6 # short moving segment, so the robot drives instead of service-chattering
GOAL_REPLAN_INTERVAL_SEC = 12.0 # give the open-loop controller time to move before another goal command
GOAL_ANCHOR_CHECK_INTERVAL_SEC = 0.75 # watch for robots drifting away after the goal has been seen
GOAL_ANCHOR_REPLAN_DISTANCE_M = 1.2 # [m] beyond this, goal memory should pull the robot back
GOAL_DIVERGENCE_REPLAN_DELTA_M = 0.25 # [m] distance increase that means the robot is wandering away
GOAL_COMPLETION_ACCEPT_RADIUS_M = 1.1 # [m] path-consumed is only an arrival hint when still near the goal
FINAL_PATH_MIN_GOAL_APPROACH_SEC = 6.0 # let the robot visibly commit before the stop message
FINAL_PATH_MAX_GOAL_APPROACH_SEC = 18.0 # don't let a stuck approach hide the returned answer forever
FINAL_PATH_GOAL_CLOSE_RADIUS = 0.45 # [m] close enough for the video/demo handoff
FINAL_PATH_ESTIMATE_TIMEOUT_SEC = 24.0 # after this, log an estimated fallback if A* never connects
PATH_SIMPLIFY_MAX_LOOKAHEAD = 24 # cells; keeps A* path intent while cutting tiny steering chops
A_STAR_MAX_EXPANSIONS = 25000 # bounding: final-path search should not freeze the ROS node
A_STAR_TIME_BUDGET_SEC = 0.35 # planning breathing: keep callbacks responsive during the demo
SMOOTHING_KERNEL_SIZE = 10  # the kernel size applied to the gaussian smoothing algorithm
SMOOTHING_SIGMA = 6 # The standard deviation applied to the gaussian smoothing algorithm

MAP_CLEAR_THRESHOLD = 33 # program treats any value below this as free space
MAP_OCCUPIED_THRESHOLD = 80 # program treats any value above this as occupied and to be avoided

CLUSTER_CELL_RADIUS = 5 # radius in cells to group frontier cells into clusters
FRONTIER_RAYCAST_WEIGHT = 0.5 # weight for unknown cells visible in score equation
FRONTIER_RAYCAST_RANGE_CELLS = 30 # max range in cells for raycast simulation
FRONTIER_RAYCAST_ANGULAR_RESOLUTION = 10 # degrees between rays in raycast simulation
# Topic names

# Frequency at which the loop operates
FREQUENCY = 5 #Hz.

USE_SIM_TIME = True
STARTUP_TIMEOUT = 15.0 # s. Max wait for simulator/controller startup.


class Coordinator(Node):
    def __init__(self, 
                 node_name="coordinator", 
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


        ## top level parameters ##
        self.num_active_robots = 0 # the number of robots that this node is coordinating
        
        self.subscription_dictionary = {} # This dictionary takes an ID of a robot and gives a set of subscribers that listen to that robot's data steam (for pose, slam_map, etc.)
        self.path_publishers_dictionary ={} # this dictionary takes an id of a robot and gives a publisher object to publish a path generated for the robot of that ID 
        self.stop_publishers_dictionary = {} # mission stopping: coordinator can hold cmd_vel at zero after final answer

        self.map_msgs = {} # dictionary that stores robot_id --> most recent occupancy grid map msg received for that robot
        self.pose_msgs = {} # dictionary that stores robot_id --> most recent pose msg received for that robot
        self.start_pose_msgs = {} # robot_id --> first pose we saw, used as that robot's start point
        self.ids_active = {} # dictionary that stores robot_id --> bool for if the robot is active or not
        self.heuristic_target_msgs = {} # robot_id --> most recent heuristic clue point from CV
        self.goal_target_msgs = {} # robot_id --> most recent goal point from CV
        self.shared_goal_target_msg = None # most recent goal, reused by robots that did not see it themselves
        self.goal_first_seen_wall_times = {} # robot_id --> first wall-clock time the goal was seen
        self.goal_approach_started_wall_times = {} # robot_id --> first time we actually sent a goal path
        self.goal_path_completion_wall_times = {} # robot_id --> mapper requested a new path after finishing a goal path
        self.goal_seen_logged = set()
        self.next_goal_replan_wall_time = {}
        self.next_goal_anchor_check_wall_time = {}
        self.last_goal_distance_m = {}
        self.next_goal_anchor_log_wall_time = 0.0
        self.last_plan_kind = {} # robot_id --> label for the kind of plan last published
        self.final_path_msg = None
        self.final_path_robot_id = None
        self.final_path_length_m = None
        self.next_final_path_attempt_wall_time = 0.0
        self.next_final_path_status_time = self.get_clock().now()
        self.next_goal_approach_status_wall_time = 0.0
        self.last_path_msgs = {}
        self.next_nav_path_republish_wall_time = 0.0
        self.current_frontiers = {} # robot_id --> frontier cell currently being investigated

        ## setting up unique ID service ##
        self.is_srv = self.create_service(GetUniqueID, 'get_unique_id', self.handle_id_request)
        self.global_id = 0  # define a global ID tracker (current global id is the most recent id assigned)

        ## setting up a path generation service ##
        self.path_srv = self.create_service(GetNewFrontierPath, 'get_path', self.handle_path_request)

        ## setting up new_robot_id topic
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.new_robot_id_publisher = self.create_publisher(Int32, "/new_robot_id", 1)
        self.final_path_publisher = self.create_publisher(PoseArray, "/final_goal_to_start_path", latched_qos)
        self.mission_complete_publisher = self.create_publisher(Bool, "/mission_complete", 1)

        ## setting up subscriber to the merged_map topic 
        # right now the merged map always treats robot_1 
        self.merged_map_sub = self.create_subscription(OccupancyGrid, "merged_map", self._merged_map_callback, 1)
        self.merged_map_info = None
        self.merged_map = None

    def handle_id_request(self, request, response):
        """This is the callback function to handle the server side of the GetUniqueID service"""
        self.global_id +=1 
        # assigning response parameters
        response.id = self.global_id
        response.success = True
        response.message = f'Assigned ID {self.global_id} to {request.requester_name}'

        # updating internally to track the number of robots we're coordinating
        self.num_active_robots +=1
        self.setup_listeners(self.global_id)
        self.setup_publishers(self.global_id)
        self.current_frontiers[self.global_id] = None

        # publishing the new id to the /new_robot_id topic
        new_id_msg = Int32()
        new_id_msg.data = self.global_id
        self.new_robot_id_publisher.publish(new_id_msg)

        return response
    
    def handle_path_request(self, request, response):
        """this function callback handles the server side of the GetNewFrontierPath Service"""
        # getting request data
        self.get_logger().info(f"received path generation request from {request.requester_name}")
        requester_id = request.requester_id
        self.get_logger().info(f"requester_name: {request.requester_name}, requester_id: {requester_id}")

        #  if id is invalid
        if requester_id <= 0 or requester_id is None:
            response.success = False
            response.message = "Requester has no id"
            self.get_logger().warn(f"received path request from robot with no ID {requester_id}")
            return response

        # generating path
        if self.last_plan_kind.get(requester_id) == "goal" and requester_id in self.goal_approach_started_wall_times:
            goal_distance = self.get_robot_goal_distance(requester_id)
            if goal_distance is None or goal_distance <= GOAL_COMPLETION_ACCEPT_RADIUS_M:
                #  arrival noting: mapper asks again after consuming the goal approach path.
                if requester_id not in self.goal_path_completion_wall_times:
                    self.get_logger().info(f"robot_{requester_id} completed a goal approach path")
                self.goal_path_completion_wall_times[requester_id] = time.monotonic()
                self.update_and_publish_final_goal_path(force=True)
                if self.final_path_msg is not None:
                    response.success = True
                    response.message = "final path acquired; mission complete"
                    return response
                response.success = False
                response.message = "goal approach recorded; waiting for final path gate"
                return response

            #  goal anchoring: path was consumed, but the robot is still too far away.
            self.goal_path_completion_wall_times.pop(requester_id, None)
            self.next_goal_replan_wall_time[requester_id] = 0.0
            self.get_logger().info(
                f"robot_{requester_id} consumed goal path but is still "
                f"{goal_distance:.2f} m from the goal; re-anchoring"
            )

        result = self.single_robot_plan(requester_id)

        # sending response
        if result == False:
            response.success = False 
            response.message = "No planner path could be found from the latest map/pose data"
        else:
            response.success = True 
            plan_kind = self.last_plan_kind.get(requester_id, "frontier")
            response.message = f"{plan_kind} path published in nav_path_{requester_id}"
        
        return response



    def _wait_for_sim_ready(self, timeout_sec):
        """Wait until simulation clock and cmd_vel subscriber are ready."""
        self.get_logger().info('Waiting for simulation to be ready...')
        start_time = time.monotonic()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - start_time >= timeout_sec:
                self.get_logger().warn('Startup wait timeout reached. Continuing anyway.')
                return
            if USE_SIM_TIME and self.get_clock().now().nanoseconds > 0:
                self.get_logger().info('Simulation ready.')
                return
    
    
    def start(self):
        """Wait for startup readiness and begin timer-driven control loop."""
        self._wait_for_sim_ready(STARTUP_TIMEOUT)
        #  wall ticking: planner republishing should not depend on Gazebo clock health.
        self._control_timer = self.create_timer(1.0 / FREQUENCY, self._control_loop_callback, clock=Clock(clock_type=ClockType.STEADY_TIME))


    ### setting up publishers and listeners for data per robot ###
    def setup_publishers(self, robot_id):
        """setus up all the publishers for this robot"""
        # path publisher
        path_publisher = self.create_publisher(PoseArray, f"nav_path_{robot_id}", 1)
        self.path_publishers_dictionary[robot_id] = path_publisher
        self.stop_publishers_dictionary[robot_id] = self.create_publisher(Twist, f"/robot{robot_id}/cmd_vel", 1)


    def setup_listeners(self, robot_id):
        """creates listeners for the various subscirptions for a given robot id (stores in subsciription_dictionary[id])"""
        poseStamped_sub = self.create_subscription(PoseStamped, f"pose_{robot_id}", functools.partial(self._pose_callback, robot_id=robot_id), 1)

        occupancyGrid_sub = self.create_subscription(OccupancyGrid, f"SLAM_map_{robot_id}", functools.partial(self._map_callback, robot_id=robot_id), 1)

        id_active_sub = self.create_subscription(Bool, f"id_active_{robot_id}", functools.partial(self._id_active_callback, robot_id=robot_id), 1)

        heuristic_sub = self.create_subscription(PointStamped, f"/robot{robot_id}/heuristic_point_odom", functools.partial(self._target_callback, robot_id=robot_id, target_kind="heuristic"), 1)

        goal_sub = self.create_subscription(PointStamped, f"/robot{robot_id}/goal_point_odom", functools.partial(self._target_callback, robot_id=robot_id, target_kind="goal"), 1)

        self.subscription_dictionary[robot_id] = (poseStamped_sub, occupancyGrid_sub, id_active_sub, heuristic_sub, goal_sub)



    def _pose_callback(self, msg:PoseStamped, robot_id:int):
        """updates the pose data coming in from robot #id"""
        self.pose_msgs[robot_id] = msg
        if robot_id not in self.start_pose_msgs:
            # Demo accounting: the first pose is the start we compare against later.
            self.start_pose_msgs[robot_id] = msg
            self.get_logger().info(
                f"stored robot_{robot_id} start pose "
                f"({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})"
            )
    
    def _map_callback(self, msg:OccupancyGrid, robot_id:int):
        """updates the map data coming in from robot #id"""
        self.map_msgs[robot_id] = msg

    def _id_active_callback(self, msg:Bool, robot_id):
        """updates the ids_active dictionary with this robot id and status"""
        # update the ids_active dictionary
        self.ids_active[robot_id] = msg.data
        # update the number of active robots
        num_active_robots = 0
        for id in self.ids_active.keys():
            if self.ids_active[id]:
                num_active_robots+=1
        self.num_active_robots = num_active_robots

    def _target_callback(self, msg:PointStamped, robot_id:int, target_kind:str):
        """Store target observations so planner requests can use the latest CV hint."""
        if target_kind == "goal":
            self.goal_target_msgs[robot_id] = msg
            self.shared_goal_target_msg = msg
            self.goal_first_seen_wall_times.setdefault(robot_id, time.monotonic())
            if robot_id not in self.goal_seen_logged:
                #  goal noticing: one log is enough, the camera publishes this fast.
                self.get_logger().info(
                    f"goal observation from robot_{robot_id}: "
                    f"({msg.point.x:.2f}, {msg.point.y:.2f})"
                )
                self.goal_seen_logged.add(robot_id)
            self.plan_goal_for_all_ready_robots()
        else:
            self.heuristic_target_msgs[robot_id] = msg
        self.update_and_publish_final_goal_path()

    def get_goal_target_for_robot(self, robot_id):
        """Return this robot's own goal, or the shared demo goal if another robot saw it."""
        return self.goal_target_msgs.get(robot_id, self.shared_goal_target_msg)

    def get_robot_goal_distance(self, robot_id):
        """Return odom distance from a robot pose to its remembered goal."""
        pose_msg = self.pose_msgs.get(robot_id)
        goal_msg = self.get_goal_target_for_robot(robot_id)
        if pose_msg is None or goal_msg is None:
            return None

        return self.euclidean_distance(
            (pose_msg.pose.position.x, pose_msg.pose.position.y),
            (goal_msg.point.x, goal_msg.point.y),
        )

    def goal_anchor_requests_replan(self, robot_id, wall_now):
        """Detect goal drift and request a fresh goal-directed plan."""
        if wall_now < self.next_goal_anchor_check_wall_time.get(robot_id, 0.0):
            return False

        self.next_goal_anchor_check_wall_time[robot_id] = wall_now + GOAL_ANCHOR_CHECK_INTERVAL_SEC
        distance_to_goal = self.get_robot_goal_distance(robot_id)
        if distance_to_goal is None:
            return False

        previous_distance = self.last_goal_distance_m.get(robot_id)
        self.last_goal_distance_m[robot_id] = distance_to_goal

        if robot_id in self.goal_path_completion_wall_times and distance_to_goal > GOAL_COMPLETION_ACCEPT_RADIUS_M:
            #  completion revoking: the robot finished a path, but it has drifted out again.
            self.goal_path_completion_wall_times.pop(robot_id, None)

        diverging = (
            previous_distance is not None
            and distance_to_goal > GOAL_ANCHOR_REPLAN_DISTANCE_M
            and distance_to_goal - previous_distance > GOAL_DIVERGENCE_REPLAN_DELTA_M
        )
        still_far_after_frontier = (
            self.last_plan_kind.get(robot_id) != "goal"
            and distance_to_goal > GOAL_ANCHOR_REPLAN_DISTANCE_M
        )

        if not diverging and not still_far_after_frontier:
            return False

        if wall_now >= self.next_goal_anchor_log_wall_time:
            #  goal remembering: if search drifts away, pull back to the last seen sphere.
            self.get_logger().info(
                f"goal anchor replan for robot_{robot_id}: "
                f"distance_to_goal={distance_to_goal:.2f} m"
            )
            self.next_goal_anchor_log_wall_time = wall_now + 1.5
        self.next_goal_replan_wall_time[robot_id] = 0.0
        return True

    def plan_goal_for_all_ready_robots(self):
        """Send a goal plan to every robot that has enough map/pose data."""
        if self.final_path_msg is not None:
            return

        wall_now = time.monotonic()
        for plan_robot_id in list(self.path_publishers_dictionary.keys()):
            anchor_replan = self.goal_anchor_requests_replan(plan_robot_id, wall_now)
            if not anchor_replan and wall_now < self.next_goal_replan_wall_time.get(plan_robot_id, 0.0):
                continue
            #  goal sharing: in the demo odom frames are aligned, so both robots can chase the found sphere.
            planned = self.single_robot_plan(plan_robot_id)
            if planned:
                self.next_goal_replan_wall_time[plan_robot_id] = wall_now + GOAL_REPLAN_INTERVAL_SEC
            else:
                self.next_goal_replan_wall_time[plan_robot_id] = wall_now + 0.8

    def _merged_map_callback(self, msg:OccupancyGrid):
        """Callback function for updating the local version of the merged map, updates self.merged_map_info (a MapMetaData) and self.merged_map (a 2D array) """
        self.merged_map_timestamp = msg.header.stamp

        self.merged_map_info = msg.info
        merged_map_resolution = resolution = self.merged_map_info.resolution
        merged_map_origin_x = self.merged_map_info.origin.position.x
        merged_map_origin_y = self.merged_map_info.origin.position.y
        merged_map_origin_theta = self.quaternion_to_theta(self.merged_map_info.origin.orientation)
        
        flat_arr = msg.data
        self.merged_map = np.reshape(flat_arr, (self.merged_map_info.height, self.merged_map_info.width))



    def unpack_map_msg(self, map_msg):
        """ returns a occupancy grid (2D array), resolution, x_occupancy_grid_origin, y_occupancy_grid_origin, origin_rotation, map_width, map_height, timestamp """
        timestamp = map_msg.header.stamp

        mapinfo = map_msg.info
        resolution = mapinfo.resolution
        x_occupancy_grid_origin = mapinfo.origin.position.x
        y_occupancy_grid_origin = mapinfo.origin.position.y
        theta_occupancy_grid_origin = self.quaternion_to_theta(mapinfo.origin.orientation)
        
        flat_arr = map_msg.data
        grid = np.reshape(flat_arr, (mapinfo.height, mapinfo.width))

        return grid, resolution, x_occupancy_grid_origin, y_occupancy_grid_origin, theta_occupancy_grid_origin, mapinfo.width, mapinfo.height, timestamp

    def unpack_pose_msg(self, pose_msg):
        """retuns x, y, theta, timestamp"""
        timestamp = pose_msg.header.stamp
        
        x = pose_msg.pose.position.x
        y = pose_msg.pose.position.y
        quat = pose_msg.pose.orientation
        theta = self.quaternion_to_theta(quat)

        return x, y, theta, timestamp


    ### general helper funcitons ###
    def quaternion_to_theta(self, quaternion):
        """given a quaternion, returns an orientation angle (around z axis)"""
        
        theta = math.atan2(
            2 * (quaternion.w*quaternion.z + quaternion.x*quaternion.y),
            1 - 2*(quaternion.y*quaternion.y + quaternion.z*quaternion.z)
        ) # converting to regular angle

        return theta
    
    
    def odom_to_cell(self, x_robot_odom, y_robot_odom, x_map_origin, y_map_origin, theta_map_origin, map_res):
        """Convert a point in odom/world coordinates to occupancy grid cell coordinates."""
        # translation
        dx = x_robot_odom - x_map_origin
        dy = y_robot_odom - y_map_origin

        # rotate into map frame
        x_map =  np.cos(-theta_map_origin) * dx - np.sin(-theta_map_origin) * dy
        y_map =  np.sin(-theta_map_origin) * dx + np.cos(-theta_map_origin) * dy

        # convert meters to cell indices
        cell_x = int(np.floor(x_map / map_res))
        cell_y = int(np.floor(y_map / map_res))

        return cell_x, cell_y


    def cell_to_odom(self, cell_x, cell_y, x_map_origin, y_map_origin, theta_map_origin, map_res):
        """Convert occupancy grid cell coordinates to that robot's odom/world coordinates."""
        # convert cell indices to meters in the map frame
        x_map = (cell_x + 0.5) * map_res # 0.5 to be in center of the cell
        y_map = (cell_y + 0.5) * map_res

        # rotate into odom/world frame
        x_robot_odom = np.cos(theta_map_origin) * x_map - np.sin(theta_map_origin) * y_map + x_map_origin
        y_robot_odom = np.sin(theta_map_origin) * x_map + np.cos(theta_map_origin) * y_map + y_map_origin

        return x_robot_odom, y_robot_odom

    def make_pose_array(self, robot_id, path_odom):
        """Turn an odom-space path into a PoseArray for publishing."""
        pose_arr_msg = PoseArray()
        pose_arr_msg.header.stamp = self.get_clock().now().to_msg()
        if robot_id in self.map_msgs:
            pose_arr_msg.header.frame_id = self.map_msgs[robot_id].header.frame_id
        else:
            pose_arr_msg.header.frame_id = f"robot{robot_id}/odom"

        pose_arr_msg.poses = []
        for pt in path_odom:
            pose = Pose()
            pose.position.x = pt[0]
            pose.position.y = pt[1]
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            pose_arr_msg.poses.append(pose)
        return pose_arr_msg

    def path_length(self, path_odom):
        """Return total polyline length in meters for an odom-space path."""
        if path_odom is None or len(path_odom) < 2:
            return 0.0

        total = 0.0
        for i in range(1, len(path_odom)):
            total += self.euclidean_distance(path_odom[i - 1], path_odom[i])
        return total

    def make_direct_odom_path(self, start_odom, goal_odom, steps=24):
        """Make a visible fallback path in odom coordinates."""
        if steps < 2:
            steps = 2

        path = []
        for i in range(steps):
            blend = i / (steps - 1)
            x = start_odom[0] + blend * (goal_odom[0] - start_odom[0])
            y = start_odom[1] + blend * (goal_odom[1] - start_odom[1])
            path.append((x, y))
        return path

    def make_goal_approach_odom_path(self, robot_odom, goal_odom, standoff_m=GOAL_APPROACH_RADIUS, steps=14):
        """Make a demo path that stops near the seen goal marker."""
        dx = goal_odom[0] - robot_odom[0]
        dy = goal_odom[1] - robot_odom[1]
        distance_to_goal = math.sqrt(dx * dx + dy * dy)
        if distance_to_goal <= standoff_m:
            return [robot_odom]

        scale = (distance_to_goal - standoff_m) / distance_to_goal
        approach_odom = (
            robot_odom[0] + dx * scale,
            robot_odom[1] + dy * scale,
        )
        return self.make_direct_odom_path(robot_odom, approach_odom, steps=steps)

    def clamp_cell(self, cell, map_width, map_height):
        """Keep a cell inside the map before nearest-free-cell search."""
        return (
            min(max(cell[0], 0), map_width - 1),
            min(max(cell[1], 0), map_height - 1),
        )

    def radius_to_cells(self, radius_m, map_res_m_per_cell):
        """Convert a meter radius into at least one map cell."""
        return max(1, int(math.ceil(radius_m / map_res_m_per_cell)))

    def apply_keepout_zones(self, search_map, keepout_zones, map_width, map_height, map_res_m_per_cell):
        """Block small regions around objects that should inform planning but not be hit."""
        for center_cell, radius_m in keepout_zones:
            if center_cell is None:
                continue

            #  heuristic avoiding: the bottle is a clue, not a place to drive into.
            cx, cy = self.clamp_cell(center_cell, map_width, map_height)
            radius_cells = self.radius_to_cells(radius_m, map_res_m_per_cell)
            for dy in range(-radius_cells, radius_cells + 1):
                for dx in range(-radius_cells, radius_cells + 1):
                    if dx * dx + dy * dy > radius_cells * radius_cells:
                        continue

                    x = cx + dx
                    y = cy + dy
                    if 0 <= x < map_width and 0 <= y < map_height:
                        search_map[y][x] = 100

    def get_goal_approach_cell(self, search_map, start_cell, goal_cell, map_width, map_height, map_res_m_per_cell):
        """Pick a free waypoint close to the goal marker, with a little standoff."""
        goal_cell = self.clamp_cell(goal_cell, map_width, map_height)
        min_radius = self.radius_to_cells(GOAL_APPROACH_RADIUS, map_res_m_per_cell)
        max_radius = min_radius + self.radius_to_cells(0.45, map_res_m_per_cell)
        best = None

        for radius in range(min_radius, max_radius + 1):
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    distance_from_goal = math.sqrt(dx * dx + dy * dy)
                    if distance_from_goal < min_radius or distance_from_goal > max_radius:
                        continue

                    x = goal_cell[0] + dx
                    y = goal_cell[1] + dy
                    if not (0 <= x < map_width and 0 <= y < map_height):
                        continue
                    if not (0 <= search_map[y][x] <= MAP_CLEAR_THRESHOLD):
                        continue

                    score = self.euclidean_distance(start_cell, (x, y)) + 0.25 * abs(distance_from_goal - min_radius)
                    if best is None or score < best[0]:
                        best = (score, (x, y))

        return None if best is None else best[1]

    def is_line_clear(self, search_map, start_cell, end_cell, map_width, map_height, allow_unknown=False):
        """Check a straight cell segment before allowing a visual fallback path."""
        distance_cells = max(abs(end_cell[0] - start_cell[0]), abs(end_cell[1] - start_cell[1]))
        if distance_cells <= 0:
            return True

        for step in range(distance_cells + 1):
            blend = step / distance_cells
            x = int(round(start_cell[0] + blend * (end_cell[0] - start_cell[0])))
            y = int(round(start_cell[1] + blend * (end_cell[1] - start_cell[1])))
            if not (0 <= x < map_width and 0 <= y < map_height):
                return False
            cell_value = search_map[y][x]
            if allow_unknown and cell_value == -1:
                continue
            if not (0 <= cell_value <= MAP_CLEAR_THRESHOLD):
                return False
        return True

    def make_clear_goal_approach_path(self, map, robot_odom, goal_odom, x_grid_origin, y_grid_origin, theta_grid_origin, grid_res, map_width, map_height):
        """Return a direct goal approach only when the occupancy grid says that line is clear."""
        path_odom = self.make_goal_approach_odom_path(
            robot_odom,
            goal_odom,
            steps=GOAL_APPROACH_PATH_STEPS,
        )
        if len(path_odom) <= 1:
            return path_odom

        start_cell = self.odom_to_cell(robot_odom[0], robot_odom[1], x_grid_origin, y_grid_origin, theta_grid_origin, grid_res)
        end_cell = self.odom_to_cell(path_odom[-1][0], path_odom[-1][1], x_grid_origin, y_grid_origin, theta_grid_origin, grid_res)
        search_map = self.build_obstacle_avoidance_search_map(map, grid_res)
        if self.is_line_clear(search_map, self.clamp_cell(start_cell, map_width, map_height), self.clamp_cell(end_cell, map_width, map_height), map_width, map_height, allow_unknown=True):
            return path_odom

        #  direct path refusing: a visible goal is not permission to drive through furniture.
        return None

    def simplify_path_cells(self, path_cell, search_map, map_width, map_height):
        """Compress an A* cell path into fewer safe visual waypoints."""
        if path_cell is None or len(path_cell) <= 2:
            return path_cell

        simplified = [path_cell[0]]
        anchor_index = 0
        while anchor_index < len(path_cell) - 1:
            candidate_index = min(len(path_cell) - 1, anchor_index + PATH_SIMPLIFY_MAX_LOOKAHEAD)
            while candidate_index > anchor_index + 1:
                if self.is_line_clear(search_map, path_cell[anchor_index], path_cell[candidate_index], map_width, map_height):
                    break
                candidate_index -= 1

            if candidate_index <= anchor_index:
                candidate_index = anchor_index + 1

            simplified.append(path_cell[candidate_index])
            anchor_index = candidate_index

        return simplified


    ### Code For Path Planning per robot ###
    def single_robot_plan(self, robot_id):
        """broadcasts plans for frontier exploraiton of a single robot, returns true if path was broadcasted, false if no path found"""
        if self.final_path_msg is not None:
            self.publish_final_goal_path()
            return False

        self.get_logger().info("starting map generation")

        if robot_id not in self.map_msgs:
            self.get_logger().warn(f"No map yet for robot {robot_id}")
            return False

        if robot_id not in self.pose_msgs:
            self.get_logger().warn(f"No pose yet for robot {robot_id}")
            return False

        if robot_id not in self.path_publishers_dictionary:
            self.get_logger().warn(f"No path publisher yet for robot {robot_id}")
            return False

        # first get the most recent SLAM Map for this robot
        map, grid_res, x_grid_origin, y_grid_origin, theta_grid_origin, map_width, map_height, map_timestamp = self.unpack_map_msg(self.map_msgs[robot_id])

        # next get the most recent robot's pose in the map
        x_robot_odom, y_robot_odom, theta_robot_odom, pose_timestamp = self.unpack_pose_msg(self.pose_msgs[robot_id])
        
        # now we convert the robot's odom coordinates to map cells
        x_map, y_map = self.odom_to_cell(x_robot_odom, y_robot_odom, x_grid_origin, y_grid_origin, theta_grid_origin, grid_res)

        goal_target_msg = self.get_goal_target_for_robot(robot_id)
        heuristic_target_msg = self.heuristic_target_msgs.get(robot_id)
        heuristic_cell = None
        keepout_zones = []
        if goal_target_msg is not None:
            #  goal priority: once the sphere exists, bottle hinting is done.
            heuristic_target_msg = None

        if heuristic_target_msg is not None:
            heuristic_cell = self.odom_to_cell(heuristic_target_msg.point.x, heuristic_target_msg.point.y, x_grid_origin, y_grid_origin, theta_grid_origin, grid_res)
            keepout_zones.append((heuristic_cell, HEURISTIC_KEEP_OUT_RADIUS))
            heuristic_distance_m = self.euclidean_distance(
                (x_robot_odom, y_robot_odom),
                (heuristic_target_msg.point.x, heuristic_target_msg.point.y),
            )
            if heuristic_distance_m <= HEURISTIC_NEAR_ROBOT_IGNORE_RADIUS:
                #  heuristic passing: once we are by the clue, stop orbiting it and keep searching.
                heuristic_cell = None

        path_cell = None
        path_odom = None
        if goal_target_msg is not None:
            goal_cell = self.odom_to_cell(goal_target_msg.point.x, goal_target_msg.point.y, x_grid_origin, y_grid_origin, theta_grid_origin, grid_res)
            path_odom = self.make_clear_goal_approach_path(
                map,
                (x_robot_odom, y_robot_odom),
                (goal_target_msg.point.x, goal_target_msg.point.y),
                x_grid_origin,
                y_grid_origin,
                theta_grid_origin,
                grid_res,
                map_width,
                map_height,
            )
            if path_odom is not None and len(path_odom) > 1:
                #  goal moving: visible target gets motion first; local lidar catches surprises.
                self.last_plan_kind[robot_id] = "goal"
                self.current_frontiers[robot_id] = None
                self.goal_approach_started_wall_times.setdefault(robot_id, time.monotonic())
                self.get_logger().info(f"Goal point for robot {robot_id} using clear visual approach path")
            else:
                path_odom = None
                path_cell = self.get_goal_path(map, x_map, y_map, goal_cell, map_width, map_height, grid_res)
                if path_cell is not None:
                    self.last_plan_kind[robot_id] = "goal"
                    self.current_frontiers[robot_id] = None
                    self.goal_approach_started_wall_times.setdefault(robot_id, time.monotonic())
                    self.get_logger().info(f"Goal point for robot {robot_id} using A* approach path")
                else:
                    path_odom = self.make_goal_approach_odom_path(
                        (x_robot_odom, y_robot_odom),
                        (goal_target_msg.point.x, goal_target_msg.point.y),
                        steps=GOAL_APPROACH_PATH_STEPS,
                    )
                    if path_odom is not None and len(path_odom) > 1:
                        #  Goal seeing, direct going: lidar recovery handles surprises better than orbiting the cue.
                        self.last_plan_kind[robot_id] = "goal"
                        self.current_frontiers[robot_id] = None
                        self.goal_approach_started_wall_times.setdefault(robot_id, time.monotonic())
                        self.get_logger().warn(f"Goal point for robot {robot_id} not connected in map yet; using direct visual approach path")

        if path_cell is None and path_odom is None:
            path_cell = self.get_frontier_path(map, x_map, y_map, map_width, map_height, grid_res, robot_id=robot_id, heuristic_cell=heuristic_cell, keepout_zones=keepout_zones)
            self.last_plan_kind[robot_id] = "frontier"

        if path_cell is None and path_odom is None:
            return False
        
        # convert the path to this robot's odom coordinates
        if path_odom is None:
            if self.last_plan_kind.get(robot_id) == "goal":
                simplification_map = self.build_goal_optimistic_search_map(map, grid_res)
            else:
                simplification_map = self.build_obstacle_avoidance_search_map(map, grid_res)
            #  path simplifying: the robot follows fewer waypoints, but the route still comes from A*.
            path_cell = self.simplify_path_cells(
                path_cell,
                simplification_map,
                map_width,
                map_height,
            )
            path_odom = [self.cell_to_odom(pt[0], pt[1], x_grid_origin, y_grid_origin, theta_grid_origin, grid_res) for pt in path_cell]

        # broadcast the path to the robot
        # first convert to a PoseArray message
        pose_arr_msg = self.make_pose_array(robot_id, path_odom[1:])
        # now publish
        self.last_path_msgs[robot_id] = pose_arr_msg
        self.publish_nav_path(robot_id, pose_arr_msg)
        self.get_logger().info(f"published nav path for robot_{robot_id} with {len(pose_arr_msg.poses)} waypoint(s)")
        self.update_and_publish_final_goal_path(force=True)
        return True


    def get_frontier_path(self, map, x_pos_map, y_pos_map, map_width, map_height, map_res_m_per_cell, robot_id=None, heuristic_cell=None, keepout_zones=None):
        """returns a list of map cell coordinates connecting the pos_map point with the best frontier"""
        keepout_zones = keepout_zones or []

        # getting current location
        start_point = x_pos_map, y_pos_map
        obstacle_avoidance_search_map = self.build_obstacle_avoidance_search_map(map, map_res_m_per_cell)
        self.apply_keepout_zones(obstacle_avoidance_search_map, keepout_zones, map_width, map_height, map_res_m_per_cell)

        # self.get_logger().info(obstacle_avoidance_search_map)
        # np.save('./map_data_pa4.npy', obstacle_avoidance_search_map) # saving the smoothed map

        start_point = self.get_nearest_free_cell(start_point[0], start_point[1], obstacle_avoidance_search_map, map_width, map_height) # snap to nearest free space as start point

        # getting the goal point
        ranked_frontiers = self.rank_frontiers(map, x_pos_map, y_pos_map, map_width, map_height, heuristic_cell=heuristic_cell, keepout_zones=keepout_zones, map_res_m_per_cell=map_res_m_per_cell)
        if len(ranked_frontiers) == 0:
            self.get_logger().warn("No frontiers found")
            return None
        for goal_point, _score in ranked_frontiers[:50]:
            path = self.a_star_path(obstacle_avoidance_search_map, start_point, goal_point, map_width, map_height, warn_on_failure=False)
            if path is not None:
                if robot_id is not None:
                    self.current_frontiers[robot_id] = goal_point
                    self.get_logger().info(f"frontier pt for robot_{robot_id}: {goal_point}")
                return path

        raw_search_map = self.build_raw_search_map(map)
        self.apply_keepout_zones(raw_search_map, keepout_zones, map_width, map_height, map_res_m_per_cell)
        raw_start_point = self.get_nearest_free_cell(x_pos_map, y_pos_map, raw_search_map, map_width, map_height)
        for goal_point, _score in ranked_frontiers[:50]:
            path = self.a_star_path(raw_search_map, raw_start_point, goal_point, map_width, map_height, warn_on_failure=False)
            if path is not None:
                if robot_id is not None:
                    self.current_frontiers[robot_id] = goal_point
                    self.get_logger().info(f"frontier pt for robot_{robot_id}: {goal_point}")
                self.get_logger().info("published bootstrap frontier path using raw occupancy grid")
                return path

        self.get_logger().warn("No reachable frontier found")
        return None

    def get_goal_path(self, map, x_pos_map, y_pos_map, goal_cell, map_width, map_height, map_res_m_per_cell):
        """returns a path from the robot to a detected goal point in map cells"""
        start_cell = self.clamp_cell((x_pos_map, y_pos_map), map_width, map_height)
        goal_cell = self.clamp_cell(goal_cell, map_width, map_height)

        obstacle_avoidance_search_map = self.build_obstacle_avoidance_search_map(map, map_res_m_per_cell)
        start_point = self.get_nearest_free_cell(start_cell[0], start_cell[1], obstacle_avoidance_search_map, map_width, map_height)
        approach_point = self.get_goal_approach_cell(obstacle_avoidance_search_map, start_point, goal_cell, map_width, map_height, map_res_m_per_cell)
        if approach_point is not None:
            #  goal approaching: drive near the sphere for the demo, not into its center.
            path = self.a_star_path(obstacle_avoidance_search_map, start_point, approach_point, map_width, map_height, warn_on_failure=False)
            if path is not None:
                return path

        raw_search_map = self.build_raw_search_map(map)
        raw_start_point = self.get_nearest_free_cell(start_cell[0], start_cell[1], raw_search_map, map_width, map_height)
        raw_approach_point = self.get_goal_approach_cell(raw_search_map, raw_start_point, goal_cell, map_width, map_height, map_res_m_per_cell)
        if raw_approach_point is not None:
            path = self.a_star_path(raw_search_map, raw_start_point, raw_approach_point, map_width, map_height, warn_on_failure=False)
            if path is not None:
                return path

        optimistic_search_map = self.build_goal_optimistic_search_map(map, map_res_m_per_cell)
        optimistic_start_point = self.get_nearest_free_cell(start_cell[0], start_cell[1], optimistic_search_map, map_width, map_height)
        optimistic_approach_point = self.get_goal_approach_cell(optimistic_search_map, optimistic_start_point, goal_cell, map_width, map_height, map_res_m_per_cell)
        if optimistic_approach_point is not None:
            #  goal insisting: once the sphere is seen, unknown cells should not freeze the demo.
            return self.a_star_path(optimistic_search_map, optimistic_start_point, optimistic_approach_point, map_width, map_height, warn_on_failure=False)

        return None

    def get_path_between_cells(self, map, start_cell, goal_cell, map_width, map_height, map_res_m_per_cell, use_bootstrap=True):
        """Plan between two cells, first safely, then with a lighter bootstrap grid if needed."""
        obstacle_avoidance_search_map = self.build_obstacle_avoidance_search_map(map, map_res_m_per_cell)
        start_cell = self.clamp_cell(start_cell, map_width, map_height)
        goal_cell = self.clamp_cell(goal_cell, map_width, map_height)

        start_point = self.get_nearest_free_cell(start_cell[0], start_cell[1], obstacle_avoidance_search_map, map_width, map_height)
        goal_point = self.get_nearest_free_cell(goal_cell[0], goal_cell[1], obstacle_avoidance_search_map, map_width, map_height)
        path = self.a_star_path(obstacle_avoidance_search_map, start_point, goal_point, map_width, map_height, warn_on_failure=False)
        if path is not None:
            return path

        if use_bootstrap:
            raw_search_map = self.build_raw_search_map(map)
            raw_start_point = self.get_nearest_free_cell(start_cell[0], start_cell[1], raw_search_map, map_width, map_height)
            raw_goal_point = self.get_nearest_free_cell(goal_cell[0], goal_cell[1], raw_search_map, map_width, map_height)
            path = self.a_star_path(raw_search_map, raw_start_point, raw_goal_point, map_width, map_height, warn_on_failure=False)
            if path is not None:
                return path

            optimistic_search_map = self.build_goal_optimistic_search_map(map, map_res_m_per_cell)
            optimistic_start_point = self.get_nearest_free_cell(start_cell[0], start_cell[1], optimistic_search_map, map_width, map_height)
            optimistic_goal_point = self.get_nearest_free_cell(goal_cell[0], goal_cell[1], optimistic_search_map, map_width, map_height)
            #  final routing: still A*, but unknown space is allowed after the demo goal is known.
            path = self.a_star_path(optimistic_search_map, optimistic_start_point, optimistic_goal_point, map_width, map_height, warn_on_failure=False)
            if path is not None:
                return path

        return None

    def update_final_goal_path(self):
        """Choose the shortest available path from a found goal to a robot start."""
        if self.final_path_msg is not None:
            return True

        best = None
        best_estimated = None
        now = self.get_clock().now()
        if self.global_id > 0:
            if len(self.start_pose_msgs) < self.global_id or len(self.map_msgs) < self.global_id:
                if now.nanoseconds >= self.next_final_path_status_time.nanoseconds:
                    #  final path waiting: both robots need starts and maps before choosing the closest start.
                    self.get_logger().info(
                        "final path waiting for demo inputs "
                        f"(robots={self.global_id}, starts={len(self.start_pose_msgs)}, "
                        f"maps={len(self.map_msgs)})"
                    )
                    self.next_final_path_status_time = now + Duration(seconds=2.0)
                return False

        if not self.goal_approach_ready():
            return False

        for robot_id in set(list(self.map_msgs.keys()) + list(self.start_pose_msgs.keys())):
            goal_msg = self.get_goal_target_for_robot(robot_id)
            if goal_msg is None:
                continue
            if robot_id not in self.map_msgs or robot_id not in self.start_pose_msgs:
                continue

            map, grid_res, x_grid_origin, y_grid_origin, theta_grid_origin, map_width, map_height, _ = self.unpack_map_msg(self.map_msgs[robot_id])
            start_msg = self.start_pose_msgs[robot_id]

            goal_cell = self.odom_to_cell(goal_msg.point.x, goal_msg.point.y, x_grid_origin, y_grid_origin, theta_grid_origin, grid_res)
            start_cell = self.odom_to_cell(start_msg.pose.position.x, start_msg.pose.position.y, x_grid_origin, y_grid_origin, theta_grid_origin, grid_res)
            goal_odom = (goal_msg.point.x, goal_msg.point.y)
            start_odom = (start_msg.pose.position.x, start_msg.pose.position.y)

            # Final path going: goal -> start, because that is what the demo statement asks to return.
            path_cell = self.get_path_between_cells(map, goal_cell, start_cell, map_width, map_height, grid_res, use_bootstrap=True)
            if path_cell is None:
                #  path estimating: the map is not fully connected yet, but the demo still needs the returned answer.
                path_odom = self.make_direct_odom_path(
                    goal_odom,
                    start_odom,
                )
                path_kind = "estimated"
            else:
                path_odom = [self.cell_to_odom(pt[0], pt[1], x_grid_origin, y_grid_origin, theta_grid_origin, grid_res) for pt in path_cell]
                path_kind = "planner"

            length_m = self.path_length(path_odom)
            candidate = {
                "robot_id": robot_id,
                "length_m": length_m,
                "path_odom": path_odom,
                "path_kind": path_kind,
                "goal_odom": goal_odom,
                "start_odom": start_odom,
            }
            if path_kind == "planner":
                if best is None or length_m < best["length_m"]:
                    best = candidate
            elif best_estimated is None or length_m < best_estimated["length_m"]:
                best_estimated = candidate

        if best is None and best_estimated is not None:
            first_goal_wall = min(self.goal_first_seen_wall_times.values()) if self.goal_first_seen_wall_times else time.monotonic()
            if time.monotonic() - first_goal_wall >= FINAL_PATH_ESTIMATE_TIMEOUT_SEC:
                best = best_estimated
            elif now.nanoseconds >= self.next_final_path_status_time.nanoseconds:
                #  A star waiting: final answer should be map-planned if the grid can connect in time.
                self.get_logger().info("final path waiting for A* goal-to-start route")
                self.next_final_path_status_time = now + Duration(seconds=2.0)
                return False

        if best is None:
            if now.nanoseconds >= self.next_final_path_status_time.nanoseconds:
                #  final path checking: target exists, but no robot has enough map/path data yet.
                self.get_logger().info("final path waiting for a usable target-to-start candidate")
                self.next_final_path_status_time = now + Duration(seconds=2.0)
            return False

        if self.final_path_length_m is not None and best["length_m"] >= self.final_path_length_m - 0.05:
            return True

        self.final_path_robot_id = best["robot_id"]
        self.final_path_length_m = best["length_m"]
        self.final_path_msg = self.make_pose_array(best["robot_id"], best["path_odom"])
        self.get_logger().info(
            f"GOAL FOUND: final {best['path_kind']} path uses robot_{best['robot_id']} start, "
            f"path_length={best['length_m']:.2f} m, topic=/final_goal_to_start_path"
        )
        self.get_logger().info(
            "FINAL PATH ACQUIRED: "
            f"closest_start_robot=robot_{best['robot_id']}, "
            f"path_kind={best['path_kind']}, "
            f"path_length_m={best['length_m']:.2f}, "
            f"waypoints={len(best['path_odom'])}, "
            f"goal_odom=({best['goal_odom'][0]:.2f}, {best['goal_odom'][1]:.2f}), "
            f"start_odom=({best['start_odom'][0]:.2f}, {best['start_odom'][1]:.2f}), "
            "path_topic=/final_goal_to_start_path, stop_topic=/mission_complete"
        )
        return True

    def goal_approach_ready(self):
        """Delay the final stop until the goal response has actually been visible."""
        if not self.goal_first_seen_wall_times:
            return True

        wall_now = time.monotonic()
        if not self.goal_approach_started_wall_times:
            if wall_now >= self.next_goal_approach_status_wall_time:
                #  approach arming: the camera saw the sphere, but the robot still needs map+pose first.
                self.get_logger().info("final path waiting for a published goal-approach command")
                self.next_goal_approach_status_wall_time = wall_now + 2.0
            return False

        first_approach_wall = min(self.goal_approach_started_wall_times.values())
        elapsed = wall_now - first_approach_wall

        arrived_distances = {}
        watched_robots = []
        for robot_id, pose_msg in self.pose_msgs.items():
            goal_msg = self.get_goal_target_for_robot(robot_id)
            if goal_msg is None:
                continue
            watched_robots.append(robot_id)
            distance_to_goal = self.euclidean_distance(
                (pose_msg.pose.position.x, pose_msg.pose.position.y),
                (goal_msg.point.x, goal_msg.point.y),
            )
            if distance_to_goal <= FINAL_PATH_GOAL_CLOSE_RADIUS or robot_id in self.goal_path_completion_wall_times:
                arrived_distances[robot_id] = distance_to_goal

        if watched_robots and len(arrived_distances) == len(watched_robots) and elapsed >= FINAL_PATH_MIN_GOAL_APPROACH_SEC:
            summary = ", ".join(
                f"robot_{robot_id}={arrived_distances[robot_id]:.2f} m"
                for robot_id in sorted(arrived_distances.keys())
            )
            #  final answer releasing: all robots with a pose have made the goal approach.
            self.get_logger().info(f"goal approach complete for all ready robots; {summary}")
            return True

        if elapsed >= FINAL_PATH_MAX_GOAL_APPROACH_SEC:
            if len(arrived_distances) > 0:
                summary = ", ".join(
                    f"robot_{robot_id}={distance:.2f} m"
                    for robot_id, distance in sorted(arrived_distances.items())
                )
                self.get_logger().info(f"goal approach timeout with partial arrival; {summary}")
                return True

            if wall_now >= self.next_goal_approach_status_wall_time:
                #  final answer releasing: goal chasing had its demo time, now the path answer matters.
                self.get_logger().info(
                    "goal approach timeout reached; releasing final path so the demo completes"
                )
                self.next_goal_approach_status_wall_time = wall_now + 2.0
            return True

        if wall_now >= self.next_goal_approach_status_wall_time:
            #  goal approach waiting: keep driving before freezing the demo with mission_complete.
            self.get_logger().info(
                "final path waiting for visible goal approach "
                f"(elapsed={elapsed:.1f}s, arrived={len(arrived_distances)}/{len(watched_robots)}, "
                f"close_radius={FINAL_PATH_GOAL_CLOSE_RADIUS:.2f} m)"
            )
            self.next_goal_approach_status_wall_time = wall_now + 2.0
        return False

    def publish_final_goal_path(self):
        """Republish the final demo answer once it has been selected."""
        if self.final_path_msg is None:
            return False

        now = self.get_clock().now()
        self.final_path_msg.header.stamp = now.to_msg()
        self.final_path_publisher.publish(self.final_path_msg)

        done_msg = Bool()
        done_msg.data = True
        self.mission_complete_publisher.publish(done_msg)
        self.publish_stop_commands()
        return True

    def publish_stop_commands(self):
        """Publish zero velocity to every known robot."""
        stop_msg = Twist()
        for publisher in self.stop_publishers_dictionary.values():
            publisher.publish(stop_msg)

    def publish_nav_path(self, robot_id, pose_arr_msg):
        """Publish a robot path, refreshing the timestamp."""
        if robot_id not in self.path_publishers_dictionary:
            return False

        pose_arr_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_publishers_dictionary[robot_id].publish(pose_arr_msg)
        return True

    def update_and_publish_final_goal_path(self, force=False):
        """Update the target-to-start answer from callbacks that do not depend on a timer."""
        wall_now = time.monotonic()
        if self.goal_target_msgs and (force or wall_now >= self.next_final_path_attempt_wall_time):
            self.update_final_goal_path()
            self.next_final_path_attempt_wall_time = wall_now + 1.0
        return self.publish_final_goal_path()

    def build_obstacle_avoidance_search_map(self, map, map_res_m_per_cell):
        """Build the inflated/smoothed grid used by the A* planner."""
        smoothed_SLAM_map = self.gaussianSmoothing(map, SMOOTHING_KERNEL_SIZE, SMOOTHING_SIGMA)
        inflated_SLAM_map = self.obstacle_inflation(map, PROTECTION_RADIUS, map_res_m_per_cell)

        obstacle_avoidance_search_map = np.zeros_like(map, dtype=np.float32)
        obstacle_avoidance_search_map[map == -1] = -1  # Unknown remains unknown
        obstacle_avoidance_search_map[inflated_SLAM_map == 100] = 100 # Inflated obstacles are blocked

        free_cells = (map != -1) & (inflated_SLAM_map != 100) # free known cells get Gaussian cost
        obstacle_avoidance_search_map[free_cells] = smoothed_SLAM_map[free_cells]
        return obstacle_avoidance_search_map

    def build_raw_search_map(self, map):
        """Build a less conservative grid for bootstrap exploration."""
        raw_search_map = np.zeros_like(map, dtype=np.float32)
        raw_search_map[map == -1] = -1
        raw_search_map[map >= MAP_OCCUPIED_THRESHOLD] = 100
        free_cells = (map != -1) & (map < MAP_OCCUPIED_THRESHOLD)
        raw_search_map[free_cells] = np.minimum(map[free_cells], MAP_CLEAR_THRESHOLD - 1)
        return raw_search_map

    def build_goal_optimistic_search_map(self, map, map_res_m_per_cell):
        """Build a goal-only grid where unknown is traversable but known obstacles stay blocked."""
        inflated_SLAM_map = self.obstacle_inflation(map, PROTECTION_RADIUS, map_res_m_per_cell)
        optimistic_search_map = np.zeros_like(map, dtype=np.float32)
        optimistic_search_map[inflated_SLAM_map >= MAP_OCCUPIED_THRESHOLD] = 100
        known_free_cells = (map != -1) & (inflated_SLAM_map < MAP_OCCUPIED_THRESHOLD)
        optimistic_search_map[known_free_cells] = np.minimum(map[known_free_cells], MAP_CLEAR_THRESHOLD - 1)
        return optimistic_search_map

    def a_star_path(self, obstacle_avoidance_search_map, start_point, goal_point, map_width, map_height, warn_on_failure=True, max_expansions=A_STAR_MAX_EXPANSIONS, max_wall_time_sec=A_STAR_TIME_BUDGET_SEC):
        """Run A* on an inflated occupancy map."""
        # list of relative neighbors to a node
        neighbor_list = NEIGHBOR_LIST
        seen_cells = np.zeros((map_height, map_width), dtype=bool) # I'll be using a 2D array to keep track of seen cells: True=visited; False=unvisited
        priority_queue = [] #creating a priority queue
        counter = 0 # a counter to break ties in the prioirty queue, this will make nodes added later be weighted towards the end
        seen_cells[start_point[1]][start_point[0]] = True # marking the start point as visited
        # creating a root node at the start point
        root_node = TreeNode("root")
        root_node.x=start_point[0] 
        root_node.y=start_point[1]
        root_node.cost=0 
        
        goal_node=None
        heapq.heappush(priority_queue, (self.euclidean_distance(start_point, goal_point), counter, root_node)) # adding, root_node our start point to the priority_queue
        
        self.a_star_count = 0
        a_star_start_wall_time = time.monotonic()
        while len(priority_queue) != 0:
            self.a_star_count+=1
            if max_expansions is not None and self.a_star_count > max_expansions:
                if warn_on_failure:
                    self.get_logger().warn("Planner: A* expansion limit reached before finding a goal")
                return None
            if max_wall_time_sec is not None and self.a_star_count % 512 == 0:
                if time.monotonic() - a_star_start_wall_time > max_wall_time_sec:
                    if warn_on_failure:
                        self.get_logger().warn("Planner: A* time budget reached before finding a goal")
                    return None
            _, _, nextup = heapq.heappop(priority_queue)

            # checking if the next cell is the goal
            if nextup.y == goal_point[1] and nextup.x == goal_point[0]: # if we've found the goal point
                goal_node = nextup 
                break
            #going through all the points neighboring the nextup
            for neighbor in neighbor_list:
                x_n, y_n = neighbor[0]+nextup.x, neighbor[1]+nextup.y # getting neighbor point coordinates
                if (0<=x_n<map_width and 0<=y_n<map_height) and 0<=obstacle_avoidance_search_map[y_n][x_n]<=MAP_CLEAR_THRESHOLD: # if the neighbor point is valid & unoccupied
                    
                    if not seen_cells[y_n][x_n]: # if the neighbor isn't already visited 
                        # creating a neighbor node to add to the graph as child of nextup
                        neighbor_node = TreeNode("child")
                        neighbor_node.parent=nextup
                        neighbor_node.x=x_n
                        neighbor_node.y=y_n
                        neighbor_node.cost=nextup.cost+self.euclidean_distance((nextup.x, nextup.y), (x_n, y_n)) 

                        #assigning the prioirty queue weight of the node we're about to add
                        weight = neighbor_node.cost + obstacle_avoidance_search_map[y_n][x_n] + self.euclidean_distance((x_n,y_n), goal_point) # weigth is cost from the smoothed map plus euclidean distance
    
                        heapq.heappush(priority_queue, (weight, counter, neighbor_node))
                        counter +=1
                        seen_cells[y_n][x_n] = True # mark as visited
                        

        if goal_node is None: # if we coudn't find the goal node
            if warn_on_failure:
                self.get_logger().warn("Planner: A* search not able to find a reachable goal")
            return(None)
        
        else:# if found, we can backtrack from the goal node to the start to get the path
            a_star_node_path = [goal_node]
            while True:
                if a_star_node_path[0].name == "root": # break before we add the root node, which is the start point, the point the robot is already on
                    break
                a_star_node_path.insert(0, a_star_node_path[0].parent)
                
            a_star_path = [(n.x,n.y) for n in a_star_node_path] # we want a list of just the point values
            return a_star_path


    def rank_frontiers(self, map, x_pos_map, y_pos_map, map_width, map_height, heuristic_cell=None, keepout_zones=None, map_res_m_per_cell=None):
        """returns a list of (frontier_pt, score) sorted in lowest to highest"""
        keepout_zones = keepout_zones or []

        # do bfs from robot position on the map to get the list of the frontier points
        frontier_points = []
        seen_cells = np.zeros((map_height, map_width), dtype=bool) # I'll be using a 2D array to keep track of seen cells: True=visited; False=unvisited
        start_cell = (x_pos_map, y_pos_map)

        queue = deque()
        queue.append(start_cell)
        seen_cells[y_pos_map][x_pos_map] = True
        while len(queue) > 0:
            nextup = queue.popleft()
            if (self.is_frontier_cell(map, nextup[0], nextup[1], map_width, map_height)):
                frontier_points.append(nextup)

            for neighbor in NEIGHBOR_LIST:
                x_n, y_n = neighbor[0]+nextup[0], neighbor[1]+nextup[1] # getting neighbor point coordinates
                if (0<=x_n<map_width and 0<=y_n<map_height): # if the neighbor point is valid
                    if not seen_cells[y_n][x_n] and 0<=map[y_n][x_n]<MAP_CLEAR_THRESHOLD: # if point is unseen, explored, & unoccupied
                        queue.append((x_n, y_n)) # add neighbor to queue
                        seen_cells[y_n][x_n] = True # mark as visited

        # Frontier scoring: one raycast per cluster keeps exploration choices useful but cheap.
        clustered = np.zeros((map_height, map_width), dtype=bool)
        clusters = []
        for pt in frontier_points:
            if clustered[pt[1]][pt[0]]:
                continue

            cluster = []
            for other_pt in frontier_points:
                if self.euclidean_distance(pt, other_pt) <= CLUSTER_CELL_RADIUS:
                    cluster.append(other_pt)
                    clustered[other_pt[1]][other_pt[0]] = True

            centroid_x = int(round(sum(p[0] for p in cluster) / len(cluster)))
            centroid_y = int(round(sum(p[1] for p in cluster) / len(cluster)))
            clusters.append((centroid_x, centroid_y, pt))

        scored_frontiers = []
        for centroid_x, centroid_y, representative_pt in clusters:
            unknown_cells_visible = self.raycast_unknown_cells(centroid_x, centroid_y, map, map_width, map_height)
            score = self.score_frontier(representative_pt, map, x_pos_map, y_pos_map, heuristic_cell=heuristic_cell, keepout_zones=keepout_zones, map_res_m_per_cell=map_res_m_per_cell)
            if math.isfinite(score):
                score -= FRONTIER_RAYCAST_WEIGHT * unknown_cells_visible
                scored_frontiers.append((representative_pt, score))

        ranked_frontiers = sorted(scored_frontiers, key=lambda x: x[1]) # sorting the frontiers by the score (lowest to highest)
        return ranked_frontiers

    def raycast_unknown_cells(self, x, y, map, map_width, map_height, cluster_cell_radius=CLUSTER_CELL_RADIUS, raycast_range=FRONTIER_RAYCAST_RANGE_CELLS, angular_resolution=FRONTIER_RAYCAST_ANGULAR_RESOLUTION):
        """Simulate a 360 degree raycast from a frontier cluster and count visible unknown cells."""
        visible_unknown = set()

        for angle_deg in range(0, 360, angular_resolution):
            angle_rad = math.radians(angle_deg)
            dx = math.cos(angle_rad)
            dy = math.sin(angle_rad)

            for step in range(1, raycast_range + 1):
                rx = int(round(x + dx * step))
                ry = int(round(y + dy * step))

                if not (0 <= rx < map_width and 0 <= ry < map_height):
                    break

                cell_val = map[ry][rx]
                if cell_val >= MAP_OCCUPIED_THRESHOLD:
                    break
                if cell_val == -1:
                    visible_unknown.add((rx, ry))

        return len(visible_unknown)

    def score_frontier(self, frontier_pt, map, x_cell_robot, y_cell_robot, heuristic_cell=None, keepout_zones=None, map_res_m_per_cell=None):
        """Given a frontier point, this function outputs a score for that point"""
        # score by distance to start
        distance_to_robot = math.sqrt((frontier_pt[0]-x_cell_robot)**2 + (frontier_pt[1]-y_cell_robot)**2)

        score = distance_to_robot
        if heuristic_cell is not None:
            distance_to_hint = math.sqrt((frontier_pt[0]-heuristic_cell[0])**2 + (frontier_pt[1]-heuristic_cell[1])**2)
            score += HEURISTIC_FRONTIER_BIAS_WEIGHT * distance_to_hint

        if keepout_zones and map_res_m_per_cell is not None:
            for center_cell, radius_m in keepout_zones:
                if center_cell is None:
                    continue
                radius_cells = self.radius_to_cells(radius_m, map_res_m_per_cell)
                distance_to_keepout = self.euclidean_distance(frontier_pt, center_cell)
                if distance_to_keepout <= radius_cells:
                    return float("inf")
                if distance_to_keepout <= 2 * radius_cells:
                    score += 4.0 * (2 * radius_cells - distance_to_keepout)
        return score

    def euclidean_distance(self, p1, p2):
        """returns euclidean distance between 2 points (x,y) tuple"""
        return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
    
    def get_nearest_free_cell(self, x, y, search_map, search_map_x, search_map_y):
        """Returns nearest free cell to inputted point, searching outward in a spiral"""
        for radius in range(0, search_map.shape[0]):
            for dx in range(-radius, radius+1):
                for dy in range(-radius, radius+1):
                    if abs(dx) != radius and abs(dy) != radius:
                        continue
                    nx, ny = x + dx, y + dy
                    if (0 <= nx < search_map_x and 
                        0 <= ny < search_map_y and
                        0 <= search_map[ny][nx] <= MAP_CLEAR_THRESHOLD):
                        return nx, ny
        return x, y  

    def is_frontier_cell(self, map, x, y, map_width, map_height):
        """returns true if the inputted point is free space on SLAM map and next to -1"""
        if map[y][x] >= MAP_OCCUPIED_THRESHOLD and map[y][x] != -1 :
            return False # if occupied or unexplored, not frontier cell...
        
        # list of relative neighbors to a node
        neighbor_list = [
                         (0,1),
                (-1,0),          (1,0),
                         (0,-1)
                ]   
        
        for dx, dy in neighbor_list:
            xn = x + dx # neigbor coordiante x
            yn = y + dy # neigbor coordiante y
            if (0 <= xn < map_width and 0 <= yn < map_height): # if neighbor in bounds
                if map[yn][xn] == -1: # if unexplored cell is neighbor 
                    return True
                
        return False

    def gaussianSmoothing(self, to_smooth_array, kernel_size, sigma):
        """This will apply gaussian smoothing to the inputted 2D array and return smoothed array as output"""
        
        if to_smooth_array is None: # have to call this after occupancy grid is called
            return None
        
        height = to_smooth_array.shape[0]
        width = to_smooth_array.shape[1]

        # constructing the kernel from the gaussian blur formula https://en.wikipedia.org/wiki/Gaussian_blur
        kernel = []
        for y in range(-kernel_size//2, kernel_size//2 +1):
            row = []
            for x in range(-kernel_size//2, kernel_size//2 +1):
                row.append(1/(2*math.pi*sigma**2) * math.exp(-1*(x**2 +y**2)/(2*sigma**2)))
            kernel.append(row)
        
        kernel = np.array(kernel) # convert to np array
        kernel = kernel / np.sum(kernel)
        
        self.smoothedMap = np.zeros((height, width), dtype=np.float32)
        # now we apply the kernel to the occupancy grid to 
        for y in range(0, height):
            for x in range(0, width):
                # for each point in the occupancy grid, we need the kernel_size x kernel_size grid aroudn that sqaure
                neighborhood = []
                for y_n in range(-kernel_size//2, kernel_size//2 +1):
                    neighborhood_row = []
                    for x_n in range(-kernel_size//2, kernel_size//2+ 1):
                        if (0<=y+y_n and y+y_n <height and 0<=x+x_n and x+x_n <width):
                            if (to_smooth_array[y+y_n][x+x_n] >=0 ):
                                neighborhood_row.append(to_smooth_array[y+y_n][x+x_n])
                            else:
                                neighborhood_row.append(0)
                        else:
                            neighborhood_row.append(0)
                    neighborhood.append(neighborhood_row)

                # Element-wise multiply and sum
                neighborhood = np.array(neighborhood, dtype=np.float32)
                self.smoothedMap[y, x] = np.sum(neighborhood * kernel) # updating the smoothed map
        
        # with large sigma, the threshold values for the walls get dimmed way down, so we have to recale
        max_val = np.max(self.smoothedMap) # find teh largest value in the map

        # Scale everything proportionally
        if max_val != 0:
            scaled = (self.smoothedMap / max_val) * 100 # the largest value becomes 100, everything else is proportional
        else:
            scaled = self.smoothedMap

        scaled[self.smoothedMap == 0] = 0 # 0 stays as 0
        self.smoothedMap = scaled.astype(np.int8) # convert to integre
       

        self.smoothedMap = self.smoothedMap.astype(np.int8) #cast to make sure we cast to become integers, because OccupancyGrid topic has int8 datatype
        # now copying over teh hard 100 values from the occupancy grid to make sure sure
        for y in range(0,height):
            for x in range(0,width):
                if to_smooth_array[y][x] ==100:
                    self.smoothedMap[y][x] = 100
                if to_smooth_array[y][x] == -1: # we can't smooth over unexplored cells #TODO: we do still include unexplored pixels in the smoothing
                    self.smoothedMap[y][x] = -1
                
        return self.smoothedMap
  
    def obstacle_inflation(self, to_inflate_map, protection_radius, resolution):
        """This function returns a map with inflated obstacles so the robot doesn't collide"""
        if to_inflate_map is None:
            return None

        inflated = np.copy(to_inflate_map)
        height, width = to_inflate_map.shape

        # getting the protection radius in cells
        radius_cells = int(protection_radius//resolution + 1)
        # getting obstacle cells
        obstacle_cells = np.argwhere(to_inflate_map >=  MAP_OCCUPIED_THRESHOLD)  # rows=y, cols=x
        # inflating area around obstacle cells
        for y, x in obstacle_cells:
            for dy in range(-radius_cells, radius_cells + 1):
                for dx in range(-radius_cells, radius_cells + 1):
                    if dx*dx + dy*dy <= radius_cells**2:
                        ny = y + dy
                        nx = x + dx

                        if 0 <= ny < height and 0 <= nx < width:
                            inflated[ny][nx] = 100

        return inflated

    def _control_loop_callback(self): # will be called every self.delta_t seconds 
        wall_now = time.monotonic()
        if self.final_path_msg is None and self.last_path_msgs and wall_now >= self.next_nav_path_republish_wall_time:
            #  path rebroadcasting: late subscribers should still get a waypoint.
            for robot_id, pose_arr_msg in self.last_path_msgs.items():
                self.publish_nav_path(robot_id, pose_arr_msg)
            self.next_nav_path_republish_wall_time = wall_now + 0.5

        if self.goal_target_msgs:
            self.update_and_publish_final_goal_path()
            if self.final_path_msg is None:
                self.plan_goal_for_all_ready_robots()

        if self.final_path_msg is not None or self.num_active_robots <= 0:
            return

        for robot_id in range(1, self.global_id + 1):
            if not self.ids_active.get(robot_id, False):
                continue
            if self.last_plan_kind.get(robot_id) != "frontier":
                continue

            frontier_pt = self.current_frontiers.get(robot_id)
            if frontier_pt is None or robot_id not in self.map_msgs:
                continue

            map, _, _, _, _, width, height, _ = self.unpack_map_msg(self.map_msgs[robot_id])
            if not self.is_frontier_cell(map, frontier_pt[0], frontier_pt[1], width, height):
                # Frontier refreshing: when the chosen edge fills in, ask for the next useful edge.
                self.get_logger().info(f"old frontier for robot_{robot_id} filled in; searching for a new one")
                self.current_frontiers[robot_id] = None
                self.single_robot_plan(robot_id)
            



def main(args=None):
    """Main function."""

    # 1st. initialization of node.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    # Initialization of the class for the coordinator.
    coordinator = Coordinator()

    # navigates and maps the environment.
    try:
        coordinator.start()
        rclpy.spin(coordinator)
    except KeyboardInterrupt:
        coordinator.get_logger().error("ROS node interrupted.")
    finally:
        coordinator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    """Run the main function."""
    main()

    
