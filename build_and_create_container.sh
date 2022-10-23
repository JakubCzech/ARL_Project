#!/bin/bash
docker build -t arl .
xhost +
docker run -it --name=ARL \
--ulimit memlock=-1 \
--privileged --env="QT_X11_NO_MITSHM=1" \
-v /home/$SUDO_USER/Shared:/root/Shared:rw \
--device=/dev/dri:/dev/dri --device=/dev/video0 \
--env="DISPLAY=$DISPLAY" --network=host \
arl bash