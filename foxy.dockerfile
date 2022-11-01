FROM osrf/ros:foxy-desktop

RUN apt-get update && apt-get upgrade -y && apt-get autoremove -y
RUN apt install -y \
    libasio-dev git python3-pip \
    ros-foxy-cv-bridge ros-foxy-camera-calibration-parsers ros-foxy-gazebo-ros-pkgs

RUN pip3 install transformations

RUN mkdir -p /root/tello_ws/src
WORKDIR /root/tello_ws/src

RUN git clone https://github.com/clydemcqueen/tello_ros.git
RUN git clone https://github.com/ptrmu/ros2_shared.git

WORKDIR /root/tello_ws

RUN . /opt/ros/foxy/setup.sh && \
    colcon build --symlink-install

RUN echo "source /opt/ros/foxy/setup.sh" >> ~/.bashrc
RUN echo "source /root/tello_ws/install/setup.bash" >> ~/.bashrc