ros2 service call /drone1/tello_action tello_msgs/TelloAction "{cmd: 'land'}"
ros2 service call /drone1/tello_action tello_msgs/TelloAction "{cmd: 'takeoff'}"
ros2 service call /tello_action tello_msgs/TelloAction "{cmd: 'takeoff'}"
ros2 service call /tello_action tello_msgs/TelloAction "{cmd: 'land'}"
ros2 service call /reset_simulation std_srvs/srv/Empty
ros2 topic echo /aruco_poses
ros2 topic echo /drone1/cmd_vel
ros2 topic echo /cmd_vel
ros2 topic list
colcon build && source install/setup.bash
source install/setup.bash
ros2 launch tello_driver teleop_launch.py
ros2 launch tello_arl control_drone.launch.py
ros2 launch tello_arl simulation_1drone.launch.py
ros2 launch tello_arl simulation_2drone.launch.py
colcon build && ros2 launch tello_arl control_drone.launch.py
colcon build && ros2 launch tello_arl simulation_1drone.launch.py
colcon build && ros2 launch tello_arl simulation_2drone.launch.py
colcon build --packages-select tello_arl && ros2 launch tello_arl control_drone.launch.py
colcon build --packages-select tello_arl && ros2 launch tello_arl simulation_1drone.launch.py
colcon build --packages-select tello_arl && ros2 launch tello_arl simulation_2drone.launch.py
colcon build --packages-select tello_arl && source install/setup.bash



