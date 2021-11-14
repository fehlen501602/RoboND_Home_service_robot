#!/bin/sh

# launch turtlebot_world.launch to deploy turtlebot environment
xterm -e "cd $(pwd)/../../..;
source devel/setup.bash;
export ROBOT_INITIAL_POSE='-x -8.5 -y -4.5 -z 0 -R 0 -P 0 -Y 0';
roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/RoboND_Home_service_robot/map/home_service.world" & 

sleep 10

# launch amcl_demo.launch for localization
xterm -e "cd $(pwd)/../../..;
source devel/setup.bash;
roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/workspace/catkin_ws/src/RoboND_Home_service_robot/map/home_service.yaml" &

sleep 5

# launch view_navigation for rviz
xterm -e "cd $(pwd)/../../..;
source devel/setup.bash;
roslaunch turtlebot_rviz_launchers view_navigation.launch" &

sleep 10

# launch pick_objects node
xterm -e "cd $(pwd)/../../..;
source devel/setup.bash;
rosparam load $(pwd)/../loc/loc.yaml;
rosrun pick_objects pick_objects " &
