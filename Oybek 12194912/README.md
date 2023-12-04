gedit ~/.bashrc 
source .bashrc
printenv | grep TURTLE
gazebo
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py  # launching world envorinment on gazebo
# Open new terminal
ros2 run turtlebot3_teleop teleop_keyboard # now you will be able to navigate your robort from keyboard


ros2 topic list # you can list all topics
rqt_graph # you can see topic, nodes...


#SLAM
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True# launching RViz
#Now you can generate map scaned by Lidar sensor
mkdir maps
ros2 run nav2_map_server map_saver_cli -f maps/TeamB_SmartMobility
