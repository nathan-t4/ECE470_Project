# ECE470 Project Update 1
To move the UR3 robot and access data from the Hokuyo sensor, add the following from the *src* folder to the local catkin_ws/src  folder:
  * Copy hokuyo_test2.world to `~/catkin_netID/src/drivers/ur3_driver/worlds'
  * Copy project2.launch to `~/catkin_netID/scr/drivers/ur3_driver/launch'
  * Copy package/folder project_py to `~/catkin_ws/src'
  
To run, open the terminal and type:
  ~/cd catkin_netID
  ~/source devel/setup.bash
  ~/roslaunch ur3_driver project2.launch
  
Open another terminal to run project_exec.py:
  ~/cd catkin_netID
  ~/source devel/setup.bash
  ~/rosrun project_py project_exec.py [theta1] [theta2] [theta3] [theta4] [theta5] [theta6]

Like lab3_exec.py, project_exec.py moves the UR3 to the inputed joint angles theta1 to theta6.

Also, when running project_exec.py, the data structure "ranges" (float[]) from the hokuyo sensor are printed to the terminal.

To access "ranges", we created a new node call "project" and subscribed "project" to rostopic "spur/light/scan" (with corresponding rosmsg "/sensor_msgs/LaserScan").

To see what data types are stored in rosmsg "/sensor_msgs/LaserScan", type 
~/rosmsg info sensor_msgs/LaserScan

We chose to only access "angle_increment", "time_increment", and "ranges" at this time. The remaining data structures can be easily accessed in the same way.

The models for the warehouse robots are in the *models* folder
