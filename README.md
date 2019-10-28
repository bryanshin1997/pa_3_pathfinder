## Programming Assignment 3
# COSC 81/181
# Bryan Shin

# Purpose:
This package runs a pathfinding algorithm that, given a maze and an occupancy grid, finds a path from a given point A to a given point B. It does this using a DFS algorithm with history. 

# To run the program, follow these steps:
1. Clone this repository into a catkin_workspace. Specifically, clone it into the folder catkin_workspace/src/
2. Change directory to your catkin_workspace directory, if you are not already there.
3. Call 'catkin_make'
4. Open a new terminal and call 'source ~/catkin_workspace/devel/setup.bash
5. Change directory to catkin_workspace/src/pa_3_pathfinder/src/
6. If pathfinder.py is not already executable, make it executable by calling 'chmod +x pathfinder.py'
7. In a new terminal, call 'roscore' to start the ROS master
8. In a new terminal, call 'rosrun map_server map_server /opt/ros/kinetic/share/turtlebot_stage/maps/maze.yaml' to start the map server
9. In a new terminal, change directory to catkin_workspace/src/pa_3_pathfinder/src and call 'python pathfinder.py' to run the program.
10. To visualize the path finding, call 'rosrun rviz rviz' in a new terminal.
11. Add the topic /map and /pose_sequence to RVIZ to visualize the maze and the progression of poses.


# Notes:
1. The report for this assignment is on Canvas!
