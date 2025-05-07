# rob-ws
rob workspace
# PUT THE MAIN REPO FOLDER IN YOUR DESKTOP FOR THESE COMMANDS

# 1. Source the ROS 2 Humble environment
source /opt/ros/humble/setup.bash

# 2. Change into your workspace root on the Desktop
cd ~/Desktop/ROB-WS

# 3. Install any missing dependencies (first-time only)
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 4. Build your workspace
colcon build --symlink-install

# 5. Source your workspace overlay
source install/setup.bash

# 6. Launch the go_kart_controller package
ros2 launch go_kart_controller go_kart.launch.py

this is in linux btw :D
