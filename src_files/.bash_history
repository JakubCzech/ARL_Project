colcon build
ros2 service call /drone1/tello_action tello_msgs/TelloAction "{cmd: 'land'}"
ros2 service call /drone1/tello_action tello_msgs/TelloAction "{cmd: 'takeoff'}" 
ros2 service call /reset_simulation std_srvs/srv/Empty
colcon build && ros2 launch tello_arl simulation_1drone.launch.py
ros2 topic echo /drone1/aruco_poses
colcon build && ros2 launch tello_arl control_drone.launch.py
ros2 launch tello_driver teleop_launch.py
source install/setup.bash
export ROS_DOMAIN_ID=33
colcon build && ros2 launch tello_arl simulation_2drone.launch.py



