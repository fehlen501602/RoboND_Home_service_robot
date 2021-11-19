# RoboND Home service robot
## Home service robot
### Path planning: Pick up
![This is an image](https://github.com/fehlen501602/RoboND_Home_service_robot/blob/main/images/Pathplanning_pick%20up.png)
### Path planning: Path planning to the drop off goal
![This is an image](https://github.com/fehlen501602/RoboND_Home_service_robot/blob/main/images/Pathplanning.png)
### Path planning: Drop off
![This is an image](https://github.com/fehlen501602/RoboND_Home_service_robot/blob/main/images/Drop%20off.png)
## Packages
* **gmapping** - map the simulated environment build up for the home service robot  
* * `gmapping_demo.launch` to perform SLAM
* **AMCL(adaptive Monte Carlo localization)** - with this package, the home service robot navigate the map and localize itself by odometry and laser data  
* **turtlebot** - official ROS package with `keyboard_teleop.launch` file
* * `keyboard_teleop.launch` - manually control robot using keyboard  
* **turtlebot_interactions** - official ROS package with `view_navigation.launch` file
* * `view_navigation.launch` - observe navigation map in rviz  
* **turtlebot_simulator** - official ROS package with `turtlebot_world.launch` file  
* * `turtlebot_world.launch` - deploy turtlebot in simulated environment with specific pose  

Once the navigation goal is received, the robot plans the path using Dijkstra's algorithm and navigate to the goal
