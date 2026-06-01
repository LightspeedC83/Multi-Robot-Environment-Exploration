For making custom services etc.
https://roboticsbackend.com/ros2-create-custom-message/

the GetNewFrontierPath service will be used as follows:
- when the robot calls the service, the coordinator will generate a new path and send it over the path_`id` topic, 
- the robot will respond to the request on the service with whether the publishing this path was successful
- the robot always takes the most recent path data on this topic, this way the coordinator can override a path being taken at any time of its choosing