Python version: 3.10.12
ROS 2 Humble
Gazebo Classic


git clone https://github.com/ROBOTIS-GIT/turtlebot3.git -b humble-devel

git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git -b humble-devel

git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git -b humble-devel

'colcon build' all the above and source them

colcon build this ws

ros2 launch turtlebot3_gazebo robots_exploration_launch.py bots:={number} x_positions:="coordinates seperated by commas" y_positions:="Same" world:={world file}
ros2 launch merge_map merge_map_launch.py bots:={number}
ros2 run multi_robot_exploration control --bots {number}

After exploration is complete save the mered map in rviz and put in map folder
and build the ws
ros2 launch turtlebot3_gazebo robots_tasks_launch.py x_positions:="coordinates seperated by commas" y_positions:="Same" world:={world file}
cd task_allocation
python add_bots --bots {number}


Task allocation:
- Task can be assigned by python add_tasks.py {{x: 1.0, y:2.0}} or by publish point in rviz2
- Tasks are stored in sqlite database

python navigate.py