#!/usr/bin/env python
#The line above is important so that this file is interpreted with Python when running it.

# Author: Charles Lowney (built on code from my PA3 submission)
# Date: 5/10/26

# This Python Program implements a SLAM in a robot

# Import of python modules.
import math # use of pi.
import random # use for generating a random real number
from enum import Enum
import time
import numpy as np # for map grid representations and operations
from anytree import Node as TreeNode # for search algorithms (so important to import as Tree node because we have a Node class from ros2 already
import heapq # for A*

# import of relevant libraries.
import rclpy # module for ROS APIs
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist # message type for cmd_vel
from sensor_msgs.msg import LaserScan # message type for scan
from nav_msgs.msg import OccupancyGrid # message type for occupancyGrid
from  nav_msgs.msg import MapMetaData # for the slam_map msg.info
from geometry_msgs.msg import Pose, Point, Quaternion # for the ifnromation stored in slam_map msg.info
from tf2_ros import TransformListener, Buffer

from std_srvs.srv import SetBool # service type 


# Constants.
# Topic names
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_SCAN_TOPIC = 'base_scan' # name of laserscan topic for Stage simulator. For Gazebo, 'scan'
OCCUPANCY_GRID_TOPIC = 'nav_msgs/OccupancyGrid'
DEFAULT_SERVICE_NAME = 'on_off'

# Frequency at which the loop operates
FREQUENCY = 100 #Hz.

# Parameters
LINEAR_VELOCITY = 0.5 # m/s
ANGULAR_VELOCITY = 0.2 # rad/s

SLAM_MAP_RESOLUTION_SCALAR = 10 # number of cells in the SLAM map per meter

# Implementation of Extra Credit
PROBABILISTIC_MAPPING =  True # if this is true, instead of using binary updates, we Implement a recursive Bayesian update using log-odds to make the map resilient to sensor noise.
CELL_PROBABILITY_OCCUPIED = 70 # the minimum probability a cell has to have to be considered occupied

SCAN_DOWNSAMPLING = 1 # the robot will process every nth scan (this value is n) so 1=process every scan, 2=process every other scan...
SMOOTHING_KERNEL_SIZE = 10  # the kernel size applied to the gaussian smoothing algorithm
SMOOTHING_SIGMA = 6 # The standard deviation applied to the gaussian smoothing algorithm
MAP_CLEAR_THRESHOLD = 20 # program treats any value below this as free space

PROTECTION_RADIUS = 0.3 # [m] The radius within which the robot will stop if it detects something, then it will wait and find a new path to nearest frontier
LASER_SLEEP_TIME_AFTER_INTERRUPT = 30 # [s] the time that the robot will wait after almost running into something so that it doesn't get stuck too close 

USE_SIM_TIME = True
STARTUP_TIMEOUT = 15.0 # s. Max wait for simulator/controller startup.

class fsm(Enum):
    OFF = 0
    ON = 1
    ROTATING = 2
    MOVING = 3
    WAITING_FOR_PATH = 4
    RECOVERY = 5

class recovery_fsm(Enum):
    FINE = 0
    WAITING_FOR_PATH = 1
    EXECUTING_PATH = 2


class WorldMapper(Node):
    def __init__(self, 
                 LINEAR_VELOCITY=LINEAR_VELOCITY,
                 ANGULAR_VELOCITY=ANGULAR_VELOCITY,               
                 PROBABILISTIC_MAPPING=PROBABILISTIC_MAPPING,
                 SCAN_DOWNSAMPLING=SCAN_DOWNSAMPLING,
                 SLAM_MAP_RESOLUTION_SCALAR=SLAM_MAP_RESOLUTION_SCALAR,
                 SMOOTHING_KERNEL_SIZE=SMOOTHING_KERNEL_SIZE,
                 SMOOTHING_SIGMA=SMOOTHING_SIGMA,
                 MAP_CLEAR_THRESHOLD=MAP_CLEAR_THRESHOLD,
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

        ### Setting up publishers/subscribers. ###

        # Setting up the publisher to send velocity commands.
        self._cmd_pub = self.create_publisher(Twist, DEFAULT_CMD_VEL_TOPIC, 1)

        # Setting up subscriber receiving messages from the laser.
        self._laser_sub = self.create_subscription(LaserScan, DEFAULT_SCAN_TOPIC, self._laser_callback, 1)

        # Setting up a publisher to publish the map data the robot creates
        self._SLAM_map_pub = self.create_publisher(OccupancyGrid, '/SLAM_map', 1)
        
        # setting up on/off service
        self._on_off_service = self.create_service(SetBool, f'{node_name}/{DEFAULT_SERVICE_NAME}', self._turn_on_off_callback)
        self._fsm = fsm.ON

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

        self.frontier_path = None # This is the paht of points (in SLAM Map coordinates) to the nearest frontier node
        self.computing_frontier_path = False # so that the robot doesn't pathfind when stuck when its still pathfinding
        
        self.SMOOTHING_KERNEL_SIZE=SMOOTHING_KERNEL_SIZE
        self.SMOOTHING_SIGMA=SMOOTHING_SIGMA
        self.MAP_CLEAR_THRESHOLD=MAP_CLEAR_THRESHOLD

        self.PROTECTION_RADIUS = PROTECTION_RADIUS
        self.done_time_for_laser_wait = self.get_clock().now()
        self._recovery_fsm = recovery_fsm.FINE
        self.recovery_path_executed = False

        # parameters for local level robot rotational and translational speed
        self.LINEAR_VELOCITY = LINEAR_VELOCITY
        self.ANGULAR_VELOCITY = ANGULAR_VELOCITY

        # parameters for controlling local motion of robot
        self.done = False # if the robot has mapped its whole environment
        self.busy = False # if busy is true, then the robot is in the midst of an action
        self.action_done_time = self.get_clock().now() # this is the time at which the current action being excecuted will be finished

        # setting up transfer frames
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

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
            if clock_ready and cmd_ready:
                self.get_logger().info('Simulation ready. Node ready for activation service.')
                return
        
               

    def move(self, linear_vel, angular_vel):
        """Send a velocity command (linear vel in m/s, angular vel in rad/s)."""
        # Setting velocities.
        twist_msg = Twist()

        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        self._cmd_pub.publish(twist_msg)

    def stop(self):
        """Stop the robot."""
        twist_msg = Twist()
        self._cmd_pub.publish(twist_msg)

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
        self._wait_for_sim_ready(STARTUP_TIMEOUT)
        self._control_timer = self.create_timer(1.0 / FREQUENCY, self._control_loop_callback)


    def check_TF_buffer_has_data(self, timestamp):
        """returns True if TF buffer has data, false if empty"""
        if self._tf_buffer.can_transform(
            'rosbot/odom',         # The reference frame we are converting to (target)
            'rosbot/base_link',    # The reference frame we are converting from (source)
            timestamp
        ):
            return True
        else:
            return False

    def _laser_callback(self, msg):
        """Processing of laser message."""
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
        if (self.get_clock().now() > self.done_time_for_laser_wait) and not self.computing_frontier_path and self._recovery_fsm != recovery_fsm.EXECUTING_PATH:
            for ray_length in msg.ranges:
                if ray_length <= self.PROTECTION_RADIUS: 
                    self.get_logger().info("Found a point within protection radius, stopping and finding new path...")
                    self.stop()
                    self.busy = False
                    self.frontier_path = None
                    self._fsm = fsm.RECOVERY
                    if not self._recovery_fsm == recovery_fsm.EXECUTING_PATH:
                        self._recovery_fsm = recovery_fsm.EXECUTING_PATH
                        self.get_frontier_point(timestamp, self.MAP_CLEAR_THRESHOLD)
                        
                        duration = Duration(seconds=LASER_SLEEP_TIME_AFTER_INTERRUPT)
                        self.done_time_for_laser_wait = self.get_clock().now() + duration
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
        
        if self.PROBABILISTIC_MAPPING: # implementation of extra credit
            pass # TODO: proabalistic mapping implementation

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
        probability = 1.0 - 1.0 / (1.0 + np.exp(log_odds_map))
        occupancy = (probability * 100).astype(np.int8)
        occupancy[np.abs(log_odds_map) < 0.01] = -1
        return occupancy

    def publish_SLAM_map(self, timestamp):
        """This function publishes the current slam map """
        msg = OccupancyGrid() #creating the occupancyGrid message --the data from the occupancy grid: https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html

        msg.header.frame_id = 'rosbot/odom' # tells rviz which frame this map belongs to - TODO: not sure if this is right
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
            # get our first goal point on the frontier
            pt = self.get_frontier_point(timestamp, self.MAP_CLEAR_THRESHOLD)
            self.get_logger().info(f"first frontier point found at {pt}")



    def get_frontier_point(self, timestamp, CLEAR_THRESHOLD):
        """returns a point (in SLAM map frame) that is on the frontier of the SLAM map
            returns None if no frontier point able to be found, returns the nearest frontier
            point (x,y) in SLAM map cell coordinates. updates self.frontier_path with the path"""
        self.get_logger().info("Getting next point on frontier.")
        self.computing_frontier_path = True

        # getting current location
        x_pos_odom, y_pos_odom, theta_odom = self.get_base_link_pose_in_odom(timestamp)
        start_point = self.odom_to_cell((x_pos_odom, y_pos_odom)) 
        
        
        if self.PROBABILISTIC_MAPPING:
            SLAM_map_to_use = self.log_odds_to_occupancy(self.SLAM_map)
        else:
            SLAM_map_to_use = self.SLAM_map
            
        # smoothing the SLAM map so that we dont' run into any walls (ie. obstacle inflation)
        smoothed_SLAM_map = self.gaussianSmoothing(SLAM_map_to_use, self.SMOOTHING_KERNEL_SIZE, self.SMOOTHING_SIGMA)
        inflated_SLAM_map = self.obstacle_inflation(SLAM_map_to_use, self.PROTECTION_RADIUS, self.SLAM_map_res_m_per_cell)

        obstacle_avoidance_search_map = np.zeros_like(SLAM_map_to_use, dtype=np.float32)

        obstacle_avoidance_search_map[SLAM_map_to_use == -1] = -1  # Unknown remains unknown
        obstacle_avoidance_search_map[inflated_SLAM_map == 100] = 100 # Inflated obstacles are blocked
        
        free_cells = (SLAM_map_to_use != -1) & (inflated_SLAM_map != 100) #  free known cells get Gaussian cost
        obstacle_avoidance_search_map[free_cells] = smoothed_SLAM_map[free_cells]

        # self.get_logger().info(obstacle_avoidance_search_map)
        np.save('./map_data_pa4.npy', obstacle_avoidance_search_map) # saving the smoothed map

        start_point = self.get_nearest_free_cell(start_point[0], start_point[1], obstacle_avoidance_search_map) # snap to nearest free space as start point, needed for recovery mode

        # preforming A* to find the nearest frontier point

        # list of relative neighbors to a node
        neighbor_list = [
                         (0,1),
                (-1,0),          (1,0),
                         (0,-1)
                ]    

        best_cost = np.full((self.SLAM_MAP_SIZE_Y, self.SLAM_MAP_SIZE_X), np.inf, dtype=np.float32) # dijkstra bookepping

        seen_cells = np.zeros((self.SLAM_MAP_SIZE_Y, self.SLAM_MAP_SIZE_X), dtype=bool) # I'll be using a 2D array to keep track of seen cells: True=visited; False=unvisited
        priority_queue = [] #creating a priority_queue

        # creating a root node at the start point
        root_node = TreeNode("root")
        root_node.x=start_point[0] 
        root_node.y=start_point[1]
        root_node.cost=0 

        goal_node=None
        best_cost[start_point[1]][start_point[0]] = 0.0
        counter = 0
        heapq.heappush(priority_queue, (0.0, counter, root_node)) # adding our start point to the priority_queue
        
        while len(priority_queue) != 0:

            current_cost, _, nextup = heapq.heappop(priority_queue)
            # If already finalized, skip
            if seen_cells[nextup.y][nextup.x]:
                continue
            
            seen_cells[nextup.y][nextup.x] = True # we mark as finalized when popped

            # checking if the next cell is the goal
            if self.is_frontier_cell(SLAM_map_to_use,nextup.x, nextup.y): # if we've found a frontier point
                goal_node = nextup 
                break
            #going through all the points neighboring the nextup
            for neighbor in neighbor_list:
                x_n, y_n = neighbor[0]+nextup.x, neighbor[1]+nextup.y # getting neighbor point coordinates
                if (0<=x_n<self.SLAM_MAP_SIZE_X and 0<=y_n<self.SLAM_MAP_SIZE_Y) and 0<=obstacle_avoidance_search_map[y_n][x_n]<=CELL_PROBABILITY_OCCUPIED: # if the neighbor point is valid & unoccupied
                    if not seen_cells[y_n][x_n]: # if the neighbor isn't already visited 
                        # cost to move into this neighbor.
                        # higher map value = closer to obstacle = more expensive.
                        step_cost = 1.0 + float(obstacle_avoidance_search_map[y_n][x_n])

                        new_cost = current_cost + step_cost
                        
                        if new_cost < best_cost[y_n][x_n]: # dijkstra relaxation
                            best_cost[y_n][x_n] = new_cost

                            # creating a neighbor node to add to the graph as child of nextup
                            neighbor_node = TreeNode("child")
                            neighbor_node.parent=nextup
                            neighbor_node.x=x_n
                            neighbor_node.y=y_n
                            neighbor_node.cost = new_cost

                            counter += 1 # update tiebreaker
                            heapq.heappush(priority_queue, (new_cost, counter, neighbor_node)) # adding to pq

        if goal_node is None: # if we coudn't find the goal node
            self.get_logger().info("ALL Points Explored.")
            self.frontier_path = None
            self._fsm = fsm.OFF
            self.computing_frontier_path = False
            return None
        
        else:# if found, we can backtrack from the goal node to the start to get the path
            bfs_node_path = [goal_node]
            while True:
                bfs_node_path.insert(0, bfs_node_path[0].parent)
                if bfs_node_path[0].name == "root":
                    break
            
            self.frontier_path = [(n.x,n.y) for n in bfs_node_path] # we want a list of just the point values
            # update state machine
            if self._fsm != fsm.RECOVERY:
                self._fsm = fsm.ON

            np.save('./waypoint_path_data_pa4.npy', self.frontier_path) # saving the smoothed map
            self.computing_frontier_path = False
            return (goal_node.x, goal_node.y)


    def is_frontier_cell(self, map, x, y):
        """returns true if the inputted point is free space on SLAM map and next to -1"""
        if map[y][x] >= self.MAP_CLEAR_THRESHOLD and map[y][x] != -1 :
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
            if (0 <= xn < self.SLAM_MAP_SIZE_X and 0 <= yn < self.SLAM_MAP_SIZE_Y): # if neighbor in bounds
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
        scaled = (self.smoothedMap / max_val) * 100 # the largest value becomes 100, everything else is proportional
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
        obstacle_cells = np.argwhere(to_inflate_map >=  CELL_PROBABILITY_OCCUPIED)  # rows=y, cols=x
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
        """converts pixel/cell point values to world points"""
        origin_x = self.SLAM_map_info.origin.position.x
        origin_y = self.SLAM_map_info.origin.position.y
        res = self.SLAM_map_info.resolution

        x_world = origin_x + (cell_point[0])*res #we add the 0.5 to make the conversion map to the center of the cell, not the upper right
        y_world = origin_y + (cell_point[1])*res

        return (x_world,y_world)
    

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
        transform = self._tf_buffer.lookup_transform(
            'rosbot/odom',         # The reference frame we are converting to (target)
            'rosbot/base_link',    # The reference frame we are converting from (source)
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

    def get_laser_point_in_odom(self, timestamp, x, y):
        """This function returns (x,y,theta) position and rotation of the robot (base_link) in the odom reference frame"""
        transform = self._tf_buffer.lookup_transform(
            'rosbot/odom',         # The reference frame we are converting to (target)
            'rosbot/laser',    # The reference frame we are converting from (source)
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
        if self._fsm == fsm.OFF or self._fsm==fsm.WAITING_FOR_PATH : # if robot is turned off or waiting for path, we do nothing
            return
        
        if self._fsm == fsm.RECOVERY and  self._recovery_fsm == recovery_fsm.WAITING_FOR_PATH:
            return
        
        if self.done:
            self.stop() # stop robot
            self._fsm = fsm.OFF # turn off robot
            return
        

        if not self.busy and not self.frontier_path is None: # if we're not in the middle of an action (and we've initialized the SLAM map)
            if len(self.frontier_path) == 0: # if we are out of points in the desired path
                if self._fsm == fsm.RECOVERY:
                    self.stop()
                    self.busy = False
                    self._fsm = fsm.ON             
                    self._recovery_fsm = recovery_fsm.FINE  
                    self.frontier_path = None
                    # Immediately get a new frontier for normal exploration
                    nextPt = self.get_frontier_point(rclpy.time.Time(), self.MAP_CLEAR_THRESHOLD)
                    return

                self.frontier_path = None
                self._fsm=fsm.WAITING_FOR_PATH
                self.stop()
                self.get_logger().info("waypoint reached")
                nextPt = self.get_frontier_point(rclpy.time.Time(), self.MAP_CLEAR_THRESHOLD)
                if nextPt is None: # if we can't find a new frontier point, we're done mapping environment
                    self.done = True
                    self.stop()
                    self._fsm = fsm.OFF
                    self.get_logger().info("Environment explored")
                    return
                else:
                    self.get_logger().info("Getting nearest frontier point as waypoint")
                    self.get_logger().info(f"Next frontier point found at {nextPt}")
                    return

            
            # if there are still points that we need to get to
            self.next_point = self.frontier_path.pop(0) # get next point we need to visit
            self.next_point_world = self.cell_to_world(self.next_point) #convert the point from cell units to m

            current_point_x, current_point_y, robot_angle = self.get_base_link_pose_in_odom(rclpy.time.Time())

            # getting the angle and distance to the first point in the list
            # we now need the amount of rotation to face the next point and the distance traveled to the next point
            desired_angle = math.atan2(np.round(self.next_point_world[1]-current_point_y, decimals=2), np.round(self.next_point_world[0]-current_point_x, decimals=2))
            rotation = desired_angle - robot_angle # our desired angle
            rotation_to_next = (rotation+math.pi) % (2*math.pi) - math.pi # normalizing the angle to between pi and -pi so that we turn the least we have to
            distance_to_next = math.sqrt((self.next_point_world[0]-current_point_x)**2 + (self.next_point_world[1]-current_point_y)**2)

            #figuring out the time the rotation will take
            sleep_time = abs(rotation_to_next/self.ANGULAR_VELOCITY)
            duration = Duration(seconds=sleep_time)
            self.rotation_done_time = self.get_clock().now() + duration # updating the rotation done time
            # figuring out the time the translation will take
            sleep_time = distance_to_next/self.LINEAR_VELOCITY
            duration = Duration(seconds=sleep_time)
            self.translation_done_time = self.rotation_done_time + duration # updating the translation done time (will be executed after the rotation so we must account for that)

            #set fsm to rotating state
            self._fsm = fsm.ROTATING
            #store the angular velocity for the rotation
            self.omega = self.ANGULAR_VELOCITY 
            if rotation_to_next < 0:
                self.omega = -self.omega
            self.busy = True
        
        if self.busy: # if we are in the middle of executing an action
            #if we are busy
            if self._fsm == fsm.ROTATING: # if we are in rotation state
                self.move(0.0, self.omega) # send the rotation command
                if self.get_clock().now() > self.rotation_done_time: # if we are done, update the state to moving (move after a rotation)
                    self._fsm=fsm.MOVING

            if self._fsm == fsm.MOVING: # if we are in the movement state
                self.move(self.LINEAR_VELOCITY, 0.0) # send the move command
                if self.get_clock().now() > self.translation_done_time: # if the movement action is done, we are done with this action and we set busy to false
                    self.busy = False
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
    