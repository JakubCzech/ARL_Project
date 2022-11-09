colcon build
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/drone1
ros2 run ros2_aruco aruco_node
ros2 service call /drone1/tello_action tello_msgs/TelloAction "{cmd: 'land'}"
ros2 service call /drone1/tello_action tello_msgs/TelloAction "{cmd: 'takeoff'}" 
ros2 service call /reset_simulation std_srvs/srv/Empty
ros2 run tello_arl controller 
ros2 topic echo /drone1/aruco_poses
ros2 launch tello_gazebo simple_launch.py
ros2 launch tello_arl tello_arl.launch.py 
