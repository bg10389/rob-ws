# rob-ws
rob workspace


# 1. Source the ROS 2 Humble environment
source /opt/ros/humble/setup.bash

# 2. Change into your workspace root
cd ~/path/to/ROB-WS

# 3. (If this is your first build) install any missing dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 4. Build your workspace
colcon build --symlink-install

# 5. Source your workspace overlay
source install/setup.bash

# 6. Launch the go_kart_controller package
ros2 launch go_kart_controller go_kart.launch.py

this is in linux btw :D
