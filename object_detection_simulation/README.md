# Object Detection Simulation ROS2 Package

This package contains the simulation for a differential drive robot in a urban station world with a red box as the object to detect. Robot detects this object using the camera and then starts to move towards it. 

## Folder structure
- env-hooks : directory that contains the environment hooks required for the ignition simulation to find the models
- ign : folder that contains the gui config for setting up the ignition gui 
- launch : directory that contains the necessary launch files 
- models/object : directory that contains the object sdf and config
- models/robot : directory that contians the robot sdf and config 
- rviz : directory that contains the config to set up the rviz window
- scripts : directory that contains the script to detect object and move towards the object
- src : directory that contains the script to teleoperate the robot in the simulated world
- worlds : directory that contains the model file for the world 


## Stack used 
- Ignition/Gazebo version : Fortress
- ROS2 version : Humble Hawksbill
- Local machine OS version : Ubuntu 22.04

## Steps to run the simulation
Assuming that the local machine already has the required dependencies , here is the complete rundown of steps required to run the simulation.

- Firstly setup the package in the src folder of a workspace (lets call it simulation_ws)
```
cd ~/simulation_ws/src/object_detection_simulation
```

- Add the environment variables as given below to the ~/.bashrc file - 
```
vim ~/.bashrc
AMENT_PREFIX_PATH=/opt/ros/humble/
export IGNITION_VERSION=fortress
export IGN_CONFIG_PATH=/usr/share/ignition/
export IGN_GAZEBO_RESOURCE_PATH=/usr/share/ignition/ignition-gazebo6/
export LIBGL_ALWAYS_SOFTWARE=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

- Then build the package using the following commands - 
```
cd ~/simulation_ws/
colcon build --packages-select object_detection_simulation
```

- Then to run the simulation in the simulator - 
```
source install/setup.bash
ros2 launch object_detection_simulation simulation.launch.py
```
This launches the robot with the red box 15 m away in a urban station world and also launches the rviz window with the preset visualisation window.

- To launch the object detection logic - open another terminal in the same workspace and run the following commands -
```
cd ~/simulation_ws/
source install/setup.bash
ros2 launch object_detection_simulation detect_and_move.launch.py
```

### Side notes 

- In order to just move the robot around in the simulated world, use the following commands (the pkg must be built as shown above)- 
```
cd ~/simulation_ws/
source install/setup.bash
ros2 launch object_detection_simulation robot.launch.py
```
- Open another terminal and run these commands - 
```
cd ~/simulation_ws/
source install/setup.bash
ros2 run object_detection_simulation teleop
```

Then you can read the prompt on the terminal to move the robot around.

### Room for improvement
- 