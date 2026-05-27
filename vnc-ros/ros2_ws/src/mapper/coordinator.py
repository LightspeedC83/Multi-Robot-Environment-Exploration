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
from nav_msgs.msg import OccupancyGrid # message type for occupancyGrid
from  nav_msgs.msg import MapMetaData # for the slam_map msg.info
from geometry_msgs.msg import Pose, Point, Quaternion # for the ifnromation stored in slam_map msg.info
from tf2_ros import TransformListener, Buffer

# importing custom services
from mapper_interfaces.srv import GetUniqueID


# Constants.
# Topic names

# Frequency at which the loop operates
FREQUENCY = 100 #Hz.

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

        ## setting up unique ID service ##
        self.srv = self.create_service(GetUniqueID, 'get_unique_id', self.handle_id_request)
        self.global_id = 0  # define a global ID tracker
    
    def handle_id_request(self, request, response):
        """This is the callback function to handle the server side of the GetUniqueID service"""
        self.global_id +=1 
        # assigning response parameters
        response.id = self.global_id
        response.success = True
        response.message = f'Assigned ID {self.global_id} to {request.requester_name}'

        # updating internally to track the number of robots we're coordinating
        self.num_active_robots +=1

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


    def _control_loop_callback(self): # will be called every self.delta_t seconds 
        pass
            



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
        if rclpy.ok():
            coordinator.stop()
        coordinator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    """Run the main function."""
    main()

    


