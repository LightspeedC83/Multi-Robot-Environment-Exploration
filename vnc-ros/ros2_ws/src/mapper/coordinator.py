#!/usr/bin/env python
#The line above is important so that this file is interpreted with Python when running it.

# Author: Charles Lowney (built on code from my PA4 submission)
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
from collections import deque
import functools # for partial funciton calling

# import of relevant libraries.
import rclpy # module for ROS APIs
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import OccupancyGrid # message type for occupancyGrid
from  nav_msgs.msg import MapMetaData # for the slam_map msg.info
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Point, Quaternion # for the ifnromation stored in slam_map msg.info
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
SMOOTHING_KERNEL_SIZE = 10  # the kernel size applied to the gaussian smoothing algorithm
SMOOTHING_SIGMA = 6 # The standard deviation applied to the gaussian smoothing algorithm

MAP_CLEAR_THRESHOLD = 33 # program treats any value below this as free space
MAP_OCCUPIED_THRESHOLD = 80 # program treats any value above this as occupied and to be avoided

CLUSTER_CELL_RADIUS = 5  # radius in cells to group frontier cells into clusters
FRONTIER_RAYCAST_WEIGHT = 1  # weight for unknown cells visible in score equation
FRONTIER_RAYCAST_RANGE_CELLS = 30  # max range in cells for raycast simulation
FRONTIER_RAYCAST_ANGULAR_RESOLUTION = 10  # degrees between rays in raycast simulation

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

        self.map_msgs = {} # dictionary that stores robot_id --> most recent occupancy grid map msg received for that robot
        self.pose_msgs = {} # dictionary that stores robot_id --> most recent pose msg received for that robot
        self.ids_active = {} # dictionary that stores robot_id --> bool for if the robot is active or not

        ## setting up unique ID service ##
        self.is_srv = self.create_service(GetUniqueID, 'get_unique_id', self.handle_id_request)
        self.global_id = 0  # define a global ID tracker (current global id is the most recent id assigned)

        ## setting up a path generation service ##
        self.path_srv = self.create_service(GetNewFrontierPath, 'get_path', self.handle_path_request)
        self.current_frontiers = {} # dictionary of id --> frontier cell point to keep track of it

        ## setting up new_robot_id topic
        self.new_robot_id_publisher = self.create_publisher(Int32, "/new_robot_id", 1)

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
        self.get_logger().info(f"requester_name: {request.requester_name}, requester_id: {request.requester_id}")
        #  if id is invalid
        if requester_id <= 0 or requester_id is None:
            response.success = False
            response.message = "Requester has no id"
            self.get_logger().warn(f"received path request from robot with no ID {requester_id}")
            return response

        # generating path
        result = self.single_robot_plan(requester_id)

        # sending response
        if result == False:
            response.success = False 
            response.message = "No path able to be found to goal frontier node"
        else:
            response.success = True 
            response.message = f"path to next frontier in nav_path_{requester_id}"
        
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
        self._control_timer = self.create_timer(1.0 / FREQUENCY, self._control_loop_callback)


    ### setting up publishers and listeners for data per robot ###
    def setup_publishers(self, robot_id):
        """setus up all the publishers for this robot"""
        # path publisher
        path_publisher = self.create_publisher(PoseArray, f"nav_path_{robot_id}", 1)
        self.path_publishers_dictionary[robot_id] = path_publisher


    def setup_listeners(self, robot_id):
        """creates listeners for the various subscirptions for a given robot id (stores in subsciription_dictionary[id])"""
        poseStamped_sub = self.create_subscription(PoseStamped, f"pose_{robot_id}", functools.partial(self._pose_callback, robot_id=robot_id), 1)

        occupancyGrid_sub = self.create_subscription(OccupancyGrid, f"SLAM_map_{robot_id}", functools.partial(self._map_callback, robot_id=robot_id), 1)

        id_active_sub = self.create_subscription(Bool, f"id_active_{robot_id}", functools.partial(self._id_active_callback, robot_id=robot_id), 1)

        self.subscription_dictionary[robot_id] = (poseStamped_sub, occupancyGrid_sub, id_active_sub)



    def _pose_callback(self, msg:PoseStamped, robot_id:int):
        """updates the pose data coming in from robot #id"""
        self.pose_msgs[robot_id] = msg
    
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


    ### Code For Path Planning per robot ###
    def single_robot_plan(self, robot_id):
        """broadcasts plans for frontier exploraiton of a single robot, returns true if path was broadcasted, false if no path found"""
        self.get_logger().info("starting map generation")

        if robot_id not in self.map_msgs:
            self.get_logger().warn(f"No map yet for robot {robot_id}")
            return False

        if robot_id not in self.pose_msgs:
            self.get_logger().warn(f"No pose yet for robot {robot_id}")
            return False

        # first get the most recent SLAM Map for this robot
        map, grid_res, x_grid_origin, y_grid_origin, theta_grid_origin, map_width, map_height, map_timestamp = self.unpack_map_msg(self.map_msgs[robot_id])

        # next get the most recent robot's pose in the map
        x_robot_odom, y_robot_odom, theta_robot_odom, pose_timestamp = self.unpack_pose_msg(self.pose_msgs[robot_id])
        
        # now we convert the robot's odom coordinates to map cells
        x_map, y_map = self.odom_to_cell(x_robot_odom, y_robot_odom, x_grid_origin, y_grid_origin, theta_grid_origin, grid_res)

        # find the path to the best frontier point in cell coords
        path_cell = self.get_frontier_path(map, x_map, y_map, map_width, map_height, grid_res, robot_id)
        if path_cell is None: 
            return False
        
        # convert the path to this robot's odom coordinates
        path_odom = [self.cell_to_odom(pt[0], pt[1], x_grid_origin, y_grid_origin, theta_grid_origin, grid_res) for pt in path_cell] 

        # broadcast the path to the robot
        # first convert to a PoseArray message
        pose_arr_msg = PoseArray()
        pose_arr_msg.header.stamp = self.get_clock().now().to_msg()
        pose_arr_msg.header.frame_id = self.map_msgs[robot_id].header.frame_id # assigning the frame id as the same as received
        pose_arr_msg.poses = []
        for pt in path_odom[1:]: # skipping the first point so it's not crazy
            pose = Pose()
            pose.position.x = pt[0]
            pose.position.y = pt[1]
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            pose_arr_msg.poses.append(pose)
        # now publish
        pose_pub = self.path_publishers_dictionary[robot_id]
        pose_pub.publish(pose_arr_msg)
        self.get_logger().info(f"published nav path for robot_{robot_id}")
        return True


    def get_frontier_path(self, map, x_pos_map, y_pos_map, map_width, map_height, map_res_m_per_cell, robot_id):
        """returns a list of map cell coordinates connecting the pos_map point with the best frontier"""
        self.get_logger().info("starting frontier path generation")
        
        # getting current location
        start_point = x_pos_map, y_pos_map
        self.get_logger().info(f"start pt {start_point}")
        SLAM_map_to_use = map
            
        # smoothing the SLAM map so that we dont' run into any walls (ie. obstacle inflation)
        smoothed_SLAM_map = self.gaussianSmoothing(SLAM_map_to_use, SMOOTHING_KERNEL_SIZE, SMOOTHING_SIGMA)
        inflated_SLAM_map = self.obstacle_inflation(SLAM_map_to_use, PROTECTION_RADIUS, map_res_m_per_cell)

        obstacle_avoidance_search_map = np.zeros_like(SLAM_map_to_use, dtype=np.float32)

        obstacle_avoidance_search_map[SLAM_map_to_use == -1] = -1  # Unknown remains unknown
        obstacle_avoidance_search_map[inflated_SLAM_map == 100] = 100 # Inflated obstacles are blocked
        
        free_cells = (SLAM_map_to_use != -1) & (inflated_SLAM_map != 100) #  free known cells get Gaussian cost
        obstacle_avoidance_search_map[free_cells] = smoothed_SLAM_map[free_cells]

        # self.get_logger().info(obstacle_avoidance_search_map)
        np.save('./map_data_pa4.npy', obstacle_avoidance_search_map) # saving the smoothed map

        start_point = self.get_nearest_free_cell(start_point[0], start_point[1], obstacle_avoidance_search_map, map_width, map_height) # snap to nearest free space as start point

        # getting the goal point
        ranked_frontiers = self.rank_frontiers(obstacle_avoidance_search_map, x_pos_map, y_pos_map, map_width, map_height) 
        if len(ranked_frontiers) == 0:
            self.get_logger().warn("No frontiers found")
            return None
        goal_point = ranked_frontiers[0][0] # best frontier (one with lowest score will be first in list, and it's index 0 in the pt,score tuple)
        self.get_logger().info(f"frontier pt: {goal_point}")
        self.current_frontiers[robot_id] = goal_point # recording the frontier that this robot is actively investigating
        
        # A* to find the best path to the goal frontier point

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
        while len(priority_queue) != 0:
            self.a_star_count+=1
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
                        
        self.get_logger().info("starting finished A*")
        
        if goal_node is None: # if we coudn't find the goal node
            self.get_logger().warn("Planner: A* search not able to find a reachable goal; all frontiers explored")
            return(None)
        
        else:# if found, we can backtrack from the goal node to the start to get the path
            a_star_node_path = [goal_node]
            while True:
                if a_star_node_path[0].name == "root": # break before we add the root node, which is the start point, the point the robot is already on
                    break
                a_star_node_path.insert(0, a_star_node_path[0].parent)
                
            a_star_path = [(n.x,n.y) for n in a_star_node_path] # we want a list of just the point values
            return a_star_path


    def rank_frontiers(self, map, x_pos_map, y_pos_map, map_width, map_height):
        """returns a list of (frontier_pt, score) sorted in lowest to highest"""

        # BFS to find all frontier points reachable from robot position
        frontier_points = []
        seen_cells = np.zeros((map_height, map_width), dtype=bool)
        queue = deque()
        queue.append((x_pos_map, y_pos_map))
        seen_cells[y_pos_map][x_pos_map] = True
        while len(queue) > 0:
            nextup = queue.popleft()
            if self.is_frontier_cell(map, nextup[0], nextup[1], map_width, map_height):
                frontier_points.append(nextup)
            for neighbor in NEIGHBOR_LIST:
                x_n, y_n = neighbor[0]+nextup[0], neighbor[1]+nextup[1]
                if (0<=x_n<map_width and 0<=y_n<map_height):
                    if not seen_cells[y_n][x_n] and 0<=map[y_n][x_n]<MAP_CLEAR_THRESHOLD:
                        queue.append((x_n, y_n))
                        seen_cells[y_n][x_n] = True

        # cluster frontier points so we only raycast once per cluster
        clustered = np.zeros((map_height, map_width), dtype=bool) # tracks which frontier pts have been assigned to a cluster
        clusters = [] # list of (centroid_x, centroid_y, representative_frontier_pt)
        for pt in frontier_points:
            if clustered[pt[1]][pt[0]]:
                continue # already assigned to a cluster
            # find all frontier points within CLUSTER_CELL_RADIUS
            cluster = []
            for other_pt in frontier_points:
                if self.euclidean_distance(pt, other_pt) <= CLUSTER_CELL_RADIUS:
                    cluster.append(other_pt)
                    clustered[other_pt[1]][other_pt[0]] = True
            centroid_x = int(round(sum(p[0] for p in cluster) / len(cluster)))
            centroid_y = int(round(sum(p[1] for p in cluster) / len(cluster)))
            clusters.append((centroid_x, centroid_y, pt)) # store centroid and representative pt

        # score each cluster with one raycast, assign score to representative frontier pt
        scored_frontiers = []
        for centroid_x, centroid_y, representative_pt in clusters:
            unknown_cells_visible = self.raycast_unknown_cells(centroid_x, centroid_y, map, map_width, map_height)
            distance_to_robot = self.euclidean_distance(representative_pt, (x_pos_map, y_pos_map))
            score = distance_to_robot - FRONTIER_RAYCAST_WEIGHT * unknown_cells_visible
            scored_frontiers.append((representative_pt, score))

        ranked_frontiers = sorted(scored_frontiers, key=lambda x: x[1])
        return ranked_frontiers

    # def score_frontier(self, frontier_pt, map, x_cell_robot, y_cell_robot):
    #     """Given a frontier point, this function outputs a score for that point"""
    #     # score by distance to start
    #     distance_to_robot = math.sqrt((frontier_pt[0]-x_cell_robot)**2 + (frontier_pt[1]-y_cell_robot)**2)

    #     # find all frontier cells within cluster_cell_radius of this point (its cluster)
    #     cluster_cell_radius = CLUSTER_CELL_RADIUS
    #     map_height, map_width = map.shape
        
    #     cluster = []
    #     for dx in range(-cluster_cell_radius, cluster_cell_radius + 1):
    #         for dy in range(-cluster_cell_radius, cluster_cell_radius + 1):
    #             if dx*dx + dy*dy <= cluster_cell_radius**2:
    #                 nx, ny = frontier_pt[0] + dx, frontier_pt[1] + dy
    #                 if (0 <= nx < map_width and 0 <= ny < map_height):
    #                     if self.is_frontier_cell(map, nx, ny, map_width, map_height):
    #                         cluster.append((nx, ny))

    #     # use centroid of cluster as the representative point for raycasting
    #     if len(cluster) > 0:
    #         centroid_x = int(round(sum(p[0] for p in cluster) / len(cluster)))
    #         centroid_y = int(round(sum(p[1] for p in cluster) / len(cluster)))
    #     else:
    #         centroid_x, centroid_y = frontier_pt[0], frontier_pt[1]

    #     # raycast from centroid to estimate information gain
    #     unknown_cells_visible = self.raycast_unknown_cells(centroid_x, centroid_y, map, map_width, map_height)

    #     # lower score = better: closer frontiers with more unknown cells visible are preferred
    #     score = distance_to_robot - FRONTIER_RAYCAST_WEIGHT * unknown_cells_visible

    #     return score

    def raycast_unknown_cells(self, x, y, map, map_width, map_height, cluster_cell_radius=CLUSTER_CELL_RADIUS, raycast_range=FRONTIER_RAYCAST_RANGE_CELLS, angular_resolution=FRONTIER_RAYCAST_ANGULAR_RESOLUTION):
        """simulates a 360 degree raycast from (x,y) and returns the number of unique unknown (-1) cells visible.
        rays stop when they hit an occupied cell so cells behind obstacles are not counted."""
        visible_unknown = set() # using a set so we don't double count cells seen by multiple rays

        for angle_deg in range(0, 360, angular_resolution):
            angle_rad = math.radians(angle_deg)
            dx = math.cos(angle_rad)
            dy = math.sin(angle_rad)

            # step along the ray
            for step in range(1, raycast_range + 1):
                rx = int(round(x + dx * step))
                ry = int(round(y + dy * step))

                if not (0 <= rx < map_width and 0 <= ry < map_height): # if out of bounds stop ray
                    break

                cell_val = map[ry][rx]

                if cell_val >= MAP_OCCUPIED_THRESHOLD: # if occupied, stop ray (don't count behind obstacle)
                    break
                elif cell_val == -1: # if unknown, count it and continue (unknown space doesn't block)
                    visible_unknown.add((rx, ry))

        return len(visible_unknown)


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
        if map[y][x] >= MAP_CLEAR_THRESHOLD and map[y][x] != -1 :
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
        # checking if the current frontier has been filled in
        if self.num_active_robots <=0:
            return
        
        for id in range(1,self.global_id+1):
            if id not in self.ids_active:
                continue
            if self.ids_active[id]:
                frontier_pt = self.current_frontiers[id]
                if not frontier_pt is None:
                    # getting map for this robot
                    map, _, _, _, _, width, height, _ = self.unpack_map_msg(self.map_msgs[id])
                    if not self.is_frontier_cell(map, frontier_pt[0], frontier_pt[1], width, height): # if it's no longer a fontier point, we need to recalculate path
                        self.get_logger().info(f"Old Frontier for robot_{id} filled, in searching for new")
                        self.single_robot_plan(id)
                        self.current_frontiers[id] = None

            



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

    


