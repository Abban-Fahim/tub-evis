rm -rf build/ install/ log/
colcon build --packages-select dvs_msgs
source install/setup.zsh
colcon build
source install/setup.zsh