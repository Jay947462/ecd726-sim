# ECD 726 Car Nav2 Simulation
Overwritten from ecd726-sim to match car specs

My ecd726-sim is inside my folder called Senior_Project, it might be different for yall, this is my launch cmd


cd /mnt/c/Senior_Project
source /opt/ros/jazzy/setup.bash
colcon build --base-paths ./ecd726-sim --packages-select ecd726_description &&
source install/setup.bash &&
ros2 launch ecd726_description display.launch.py
