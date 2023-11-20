# diablo_arm
Files for the diablo robot and kinova arm integration

## Installation
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Abanesjo/diablo_arm.git
cd ..
sudo apt-get update
rosdep update --rosdistro=foxy
rosdep install --from-paths src -y --ignore-src 
colcon build
source install/setup.bash
```

## Usage
```
ros2 launch diablo_simulation gazebo.launch.py
```
You should see the robot appear in the Gazebo simulation window
