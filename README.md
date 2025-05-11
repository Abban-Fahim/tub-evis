My (almost complete) solutions to excercises from Prof. Guillermo Gallego's course ["Event-based Robot Vision"](https://sites.google.com/view/guillermogallego/teaching/event-based-robot-vision), from TUB.

MY solution for excercise 3 (`evis_ws`) is implemented in ROS2, as opposed to the the professor's choice of ROS1 and the provided code. To ensure compatibility, I've ported the `dvs_msgs` package from the RPG's [rpg_dvs_ros](https://github.com/uzh-rpg/rpg_dvs_ros) repo. To convert any ROS1 bag, including the one used in this excercise, to ROS2, install the `rosbags` library from pip and convert it using the following command (ensuring the executable are in your path):

```bash
rosbags-convert --src <ros1_bag_name>.bag --dst <ros2_bag_folder name>
```

To run this bag, you must ensure to source the workspace containing `dvs_msgs`. Example instructions to use the bag I used are as follow:

```bash
cd evis_ws
colcon build --symlink-install
source install/setup.bash
wget http://rpg.ifi.uzh.ch/datasets/davis/slider_depth.bag
rosbags-convert --src slider_depth.bag --dst slider_depth_ros2
ros2 bag play -l slider_depth_ros2/slider_depth_ros2.db3
```

<!-- check my notes  -->
