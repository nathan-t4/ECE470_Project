# Simulator

## Terminal 1
- Switch to your catkin directory
- `$ source devel/setup.bash`
- `$ roslaunch ur3_driver project.launch`
 
## Terminal 2
- Switch to your catkin directory
- `$ source devel/setup.bash`
- To pick up blocks 3 (red), 1 (green), and 2 (yellow) in order:
- `$ rosrun project_py project_exec.py --block 3 1 2`
