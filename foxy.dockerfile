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
RUN git clone https://github.com/lapo5/ROS2-Aruco-TargetTracking.git

RUN mv ROS2-Aruco-TargetTracking/ camera_target_tracking
WORKDIR /root/tello_ws

RUN . /opt/ros/foxy/setup.sh && \
    colcon build --symlink-install
ADD ./src/start.sh .
RUN cp -r /root/tello_ws/src/tello_ros/tello_gazebo/models/* ~/.gazebo/
ADD ./src_files/simple_launch.py /root/tello_ws/src/tello_ros/tello_gazebo/launch/simple_launch.py
RUN echo "source /opt/ros/foxy/setup.sh" >> ~/.bashrc
RUN echo "source /root/tello_ws/install/setup.bash" >> ~/.bashrc
RUN echo "alias start='colcon build && . install/setup.bash  && ros2 run tello_arl controller'" >> ~/.bashrc 