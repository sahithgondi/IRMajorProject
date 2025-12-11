# IRMajorProject
roslaunch turtlebot_bringup minimal.launch

roslaunch turtlebot_navigation amcl_demo.launch map_file:=/home/gond0013/my_lab_map.yaml

roslaunch turtlebot_rviz_launchers view_navigation.launch

roslaunch MajorProject major.launch

roslaunch turtlebot_teleop keyboard_teleop.launch

roslaunch MajorProject major.launch
