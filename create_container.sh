#!/bin/bash
xhost +
docker run -it --name=arl_foxy \
--ulimit memlock=-1 \
--privileged --env="QT_X11_NO_MITSHM=1" \
--device=/dev/dri:/dev/dri --device=/dev/video0 \
--device=/dev/shm:/dev/shm \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
-v $(pwd)/src_files/src:/root/tello_ws/src/:rw \
--env="DISPLAY=$DISPLAY" --network=host \
arl:foxy bash