FROM osrf/ros:foxy-desktop
LABEL org.opencontainers.image.authors="czechjakub@icloud.com"

RUN apt-get update && apt-get upgrade -y && apt-get autoremove -y
RUN apt install -y \
    libasio-dev git python3-pip \
    ros-foxy-cv-bridge ros-foxy-camera-calibration-parsers ros-foxy-gazebo-ros-pkgs

RUN pip3 install transformations opencv-contrib-python==4.7.0.68 scipy

RUN mkdir -p /root/tello_ws/src
WORKDIR /root/tello_ws

ADD ./src_files/start.sh .
ADD ./src_files/.gazebo /root/.gazebo
ADD ./src_files/.bash_history /root/.bash_history

RUN echo "source /opt/ros/foxy/setup.sh" >> ~/.bashrc
RUN echo "alias _start='colcon build && . /root/tello_ws/install/setup.bash  && ros2 launch tello_arl control_drone.launch.py '" >> ~/.bashrc 
RUN echo "alias _sim_start='colcon build && . /root/tello_ws/install/setup.bash  && ros2 launch tello_arl simulation_1drone.launch.py '" >> ~/.bashrc 

