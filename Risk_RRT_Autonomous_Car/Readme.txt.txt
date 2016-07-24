*Abstract*

Partial Motion planning will be implemented based on the tool named "Risk-RRT" algorithm to generate the risk-free trajectory. Path following Controller will be created based on the pure pursuit controller algorithm just to follow the dynamically changing path given by Risk-RRT.

*Nodes*

og_builder_socialspaces - Build occupancy grid map and update the manually genrated pedestrian and their social spaces into the grid map and considered those respective spaces as occupied.

subscribed topics: /FLUENCE/submap (nav_msgs/OccupancyGrid)
                           /FLUENCE/obstacle/filtered  (icars_object_ekf/Object)

publishedtopics: human_marker_n(visualization_msgs/MarkerArray)
                         occupancy grid array (riskrrt/OccupancyGridArray)

riskrrt_planner - plans the possible trajectories required to reach the goal and selects the best one out of it with less risk of collision.

subscribed topics: occupancy grid array (riskrrt/OccupancyGridArray)
                            goal (geometry_msgs/PoseStamped)
                            controller feedback (std_msgs/Bool)
                            robotpose (icars_2d_map_manager/Status)
                            robot odometry (nav_msgs/Odometry)

 published topics: trajectory (riskrrt/Trajectory)

pure_pursuit_controller - get the trajectories from planning algorithm and generates the required steering angle to follow the trajectory.

subscribed topics: trajectory (riskrrt/Trajectory) 
                            robot pose (icars_2d_map_manager/Status)
  
published topics: controller feedback, true if robot is on trajectory      (std_msgs/Bool)
                              velocity commands (geometry_msgs/Twist)

recording - records the possible data required so that they can be used for detailed analysis by plotting them in a graph later.

             subscribed topics: map (nav_msgs/OccupancyGrid)
                                         human odometry (nav_msgs/Odometry)
                                         robot pose (icars_2d_map_manager/Status)


*Launch*

run the scenario with
$ roslaunch riskrrt planner.launch
change the planner behavior by modifying the parameters in params/riskrrt_params.yaml