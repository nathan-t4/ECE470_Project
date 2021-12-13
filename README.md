# ECE470 Final Project 
To move the UR3 robot and access data from the Hokuyo sensor, add the following from the *src* folder to the local catkin_ws/src  folder:
  * Copy project.world to `~/catkin_netID/src/drivers/ur3_driver/worlds`
  * Copy project.launch to `~/catkin_netID/src/drivers/ur3_driver/launch`
  * Copy package/folder project_py to `~/catkin_ws/src`
  * Copy block_big_green.urdf, block_big_yellow.urdf, block_big_red.urdf, cabinet_plate.urdf & destination_plate.urdf to `~/catkin_ws/src/drivers/universal_robot/ur_description_urdf`
  
To run, open the terminal and type:
  * `~/cd catkin_netID`
  * `~/source devel/setup.bash`
  * `~/roslaunch ur3_driver project.launch`
  
Open another terminal to run project_exec.py:
  * `~/cd catkin_netID`
  * `~/source devel/setup.bash`
  * `~/rosrun project_py project_exec.py --block (blocks in order)`

The input (blocks in order) tells the UR3 which blocks and what order to move blocks from the cabinet (black) to destination (red). 
For example, `~/rosrun project_py project_exec.py --block 3 1 2` will move the red block (3), green block (1), and yellow block (2) in order.

With data from the Hokuyo sensor, forward kinematics, and inverse kinematics, `project_exec.py` will move the UR3 to the location of the big blocks.

To access the data from the sensor, we created a new node call "project" and subscribed "project" to rostopic "spur/light/scan" (with corresponding rosmsg "/sensor_msgs/LaserScan").

To see what data types are stored in rosmsg "/sensor_msgs/LaserScan", type 
`~/rosmsg info sensor_msgs/LaserScan`

With the data from the LaserScan message, `project_coordinate_converter.py` estimates the position of the sensed object. The estimated coordinates are then passed to the inverse kinematics function in 'project_func.py' to obtain the corresponding UR3 joint angles required to reach the given position. The blocks are then picked up with the gripper.

The models for the warehouse robots are in the *models* folder
