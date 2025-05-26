rm -rf build/ install/ log/
colcon build --packages-select dvs_msgs
source install/setup.bash
colcon build
source install/setup.bash