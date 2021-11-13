# ECE470 Project Update 2
To move the UR3 robot and access data from the Hokuyo sensor, add the following from the *src* folder to the local catkin_ws/src  folder:
  * Copy hokuyo_test2.world to `~/catkin_netID/src/drivers/ur3_driver/worlds`
  * Copy project.launch to `~/catkin_netID/scr/drivers/ur3_driver/launch`
  * Copy package/folder project_py to `~/catkin_ws/src`
  
To run, open the terminal and type:
  * `~/cd catkin_netID`
  * `~/source devel/setup.bash`
  * `~/roslaunch ur3_driver project.launch`
  
Open another terminal to run project_exec.py:
  * `~/cd catkin_netID`
  * `~/source devel/setup.bash`
  * `~/rosrun project_py project_exec.py [theta1] [theta2] [theta3] [theta4] [theta5] [theta6]`

With data from the Hokuyo sensor, forward kinematics, and inverse kinematics, `project_exec.py` will move the UR3 to the location of the big green block.

Also, when running project_exec.py, the position of the big green block in the world frame is printed into the terminal.

As from the previous project update, to access the data from the sensor, we created a new node call "project" and subscribed "project" to rostopic "spur/light/scan" (with corresponding rosmsg "/sensor_msgs/LaserScan").

To see what data types are stored in rosmsg "/sensor_msgs/LaserScan", type 
`~/rosmsg info sensor_msgs/LaserScan`

With the data from the LaserScan message, `project_coordinate_converter.py` estimates the position of the sensed object. The estimated coordinates are then passed to the inverse kinematics function in 'project_func.py' to obtain the corresponding UR3 joint angles required to reach the given position.

The models for the warehouse robots are in the *models* folder
