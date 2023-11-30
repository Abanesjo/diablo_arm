# diablo_arm
Files for the diablo robot and kinova arm integration. Tested on ROS2 Foxy

## Dependencies
For velocity control with teleop_twist, xterm is required. 
```
sudo apt install xterm
```

## Installation
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Abanesjo/diablo_arm.git
cd ..
sudo apt-get update
rosdep update --rosdistro=foxy
rosdep install --from-paths src -y --ignore-src 
colcon build --symlink-install
```

## Usage
To launch the simulation without the arm:
```
source install/setup.bash
ros2 launch diablo_simulation gazebo.launch.py
```
To launch the simulation with the arm:
```
source install/setup.bash
ros2 launch diablo_simulation gazebo_arm.launch.py
```
A version without plugins can also be simulated.
```
source install/setup.bash
ros2 launch diablo_simulation gazebo_noplugin.launch.py
```

You should see the robot appear in the Gazebo simulation window

![diablo_arm.png](/docs/diablo_arm.png)
