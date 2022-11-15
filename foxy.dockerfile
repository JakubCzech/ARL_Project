FROM osrf/ros:foxy-desktop
LABEL org.opencontainers.image.authors="czechjakub@icloud.com"

RUN apt-get update && apt-get upgrade -y && apt-get autoremove -y
RUN apt install -y \
    libasio-dev git python3-pip \
    ros-foxy-cv-bridge ros-foxy-camera-calibration-parsers ros-foxy-gazebo-ros-pkgs

RUN pip3 install transformations opencv-python opencv-contrib-python scipy

RUN mkdir -p /root/tello_ws/src
WORKDIR /root/tello_ws/src
ADD ./src_files/tello_arl ./tello_arl
RUN git clone https://github.com/clydemcqueen/tello_ros.git
RUN git clone https://github.com/ptrmu/ros2_shared.git
RUN git clone https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco.git

WORKDIR /root/tello_ws
# Change ros packages file to custom 
ADD ./src_files/aruco_node.py /root/tello_ws/src/ros2_aruco/ros2_aruco/ros2_aruco/aruco_node.py
ADD ./src_files/f2.world /root/tello_ws/src/tello_ros/tello_gazebo/worlds/f2.world
# ADD ./src_files/simple_launch.py /root/tello_ws/src/tello_ros/tello_gazebo/launch/simple_launch.py
ADD ./src_files/arl_project.py /root/tello_ws/src/tello_ros/tello_gazebo/launch/arl_project.py
ADD ./src_files/tello_description_arl /root/tello_ws/src/tello_ros/tello_description_arl
# Build all
RUN . /opt/ros/foxy/setup.sh && \
    colcon build --symlink-install
# Add start script
ADD ./src_files/start.sh .
ADD ./src_files/.gazebo /root/.gazebo
ADD ./src_files/.bash_history /root/.bash_history
RUN echo "source /opt/ros/foxy/setup.sh" >> ~/.bashrc
RUN echo "source /root/tello_ws/install/setup.bash" >> ~/.bashrc
RUN echo "alias _start='colcon build && . /root/tello_ws/install/setup.bash  && ros2 launch tello_arl tello_arl.launch.py '" >> ~/.bashrc 
RUN echo "alias _sim_start='colcon build && . /root/tello_ws/install/setup.bash  && ros2 launch tello_gazebo arl_project.py '" >> ~/.bashrc 

