#!/bin/bash
docker run --net=host -it --rm \
           --gpus all \
           -e "DISPLAY=$DISPLAY" \
           -e "QT_X11_NO_MITSHM=1" \
           -v "/tmp/.X11-unix:/tmp/.X11-unix" \
           -v "$HOME/.Xauthority:/root/.Xauthority:rw" \
           -v $(realpath ..):/root \
           -v /home/toyozoshimada/sandbox_ws/tum_dataset:/dataset \
           -w /root \
           $@ \
           orb_slam3