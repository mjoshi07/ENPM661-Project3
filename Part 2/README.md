# Part 2
A* path planning with non-holonomic constraints

Given a start and goal location, plan a path using A* algorithm
Robot dimensions are set to Turtlebot3 `Waffle_Pi`

## Run instructions
```bash
cd ~/catkin_ws/src
```
* Paste the ROS package astar_robot folder here 

  ```
  cd ~/catkin_ws
  python3 src/astar_robot/src/AStar.py
  ```

* The program asks the following inputs from the user:

* `clearance`

* `rpm1`

* `rpm2`

* `start x`

* `start y`

* `start theta`

* `goal x`

* `goal y`

* After this A* algorithms finds the shortest path and matplotlib visualization is performed after search is complete. Close the matplotlib window, now the execution is paused

* Next gazebo command is printed on the  screen which would be similar to the following

  ```
  source devel/setup.bash && roslaunch astar_robot astar.launch x_pos:=-2.0 y_pos:=-1.0 yaw:=0.523598775598298
  ```

* Open a new terminal and execute the above terminal commands for gazebo to launch with turtlebot spawned at start position

* Once the turtlebot gazebo environment is spawned, come back to previous terminal and resume the python code by pressing any key+enter

* At this point Turtlebot should start following the planned path.
