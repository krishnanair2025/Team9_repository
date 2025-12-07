# Navigation Workspace

This workspace contains packages related to navigation and mapping functionalities of the rover. It also contains the package for the main task manager node which orchestrates the rover's behaviour during the mission. 

Gazebo Simulation:

In the simulated gazebo world, there is an outer boundary object which simulates the 6 x 4m search area that the rover operates within. The leo rover itself has a lidar mounted on it. There are random shapes placed within the 6 x 4m area to act as obstacles. 

Auto exploration:

To launch auto exploration: ros2 launch leo_gz_bringup auto_explore.launch.py 

To activate explore server: ros2 service call /explorer_status std_srvs/srv/SetBool "{data: true}"

This will launch the gazebo simulation, SLAM toolbox, Nav2, scan_filtering_node, frontier_exploration_node and rviz2 window. The frontier exploration node repeatedly sends frontier goals to Nav2 until the whole map is explored.

Task manager:

To launch task manager simulation: ros2 launch leo_gz_bringup task_manager.launch.py

To trigger explore state: ros2 service call /trigger_rover example_interfaces/srv/Trigger "{}"

This launches gazebo simulation, SLAM toolbox, Nav2, scan_filtering_node, frontier_exploration_node, dummy_object_detection_node, task_manager, move_to_coord_node and rviz2 window. The task manager orchestrates the rover to begin in exploring mode. The dummy object detection node simulates the camera finding a coloured object using a random number generator. When the random number generator rolls 13, it publishes a boolean message and a target coordinate which represents the location of the coloured object. Upon receiving this boolean message, the task manager calls the frontier exploration service to pause exploration and triggers move_to_coord node with another service call. This node takes the coordinates published by the object detection node and move the rover to this location. Once the rover has reached the location, it sends a confirmation through another boolean message. The task manager then starts a 30s timer to simulate the arm picking the object. After the 30s timer, the task manager resumes the frontier exploration. This cycle repeats until the internal variable which stores the number of objects collected reaches 3. 

