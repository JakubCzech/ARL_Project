#!/bin/bash
# Uruchomienie symulacji drona
# ros2 launch tello_gazebo simple_launch.py

# # Wystartowanie drona nad ziemie
# ros2 service call /drone1/tello_action tello_msgs/TelloAction "{cmd: 'takeoff'}" 

# # Lądowanie drona 
# ros2 service call /drone1/tello_action tello_msgs/TelloAction "{cmd: 'land'}"

# Latanie dronem za pomocą klawiatury
# # ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/drone1

# # reset symulacji
#  ros2 service call /reset_simulation std_srvs/srv/Empty
colcon build --packages-select tello_arl && source install/setup.bash  && ros2 launch tello_arl simulation_2drone.launch.py