# cerulean_sonar
ROS driver for publishing data from cerulean sonar devices. This package simply provides an interface between ROS2 and cerulean sonar devices. 

Supported devices and what has been implemented:
S500 [Minimally Implemented] - Publishes range messages with s500 data
Rov Locator MkII [PLANNED] - 
Rov Locator MkIII [PLANNED] -

This package is in the beta stages and is still being written. There may be little functionality or no functionality.

# Install instructions
1. Create a ROS2 workspace with a src folder
2. Cd into the src folder
3. Clone this repository
4. colcon build
```
cd <ros_workspace>/src
git clone https://github.com/jackarivera/cerulean_sonar.git
cd ..
colcon build
```

# How to run
1. Make sure you source the newly installed package
2. Run using ros2 run
```
cd <ros_workspace>
source install/setup.sh
ros2 run cerulean_sonar sonar_node
```
