#!/bin/bash

if [ -z "$1" ]
  then
    echo "You should provide the ip of the master as second argument"
    exit 2
fi

xhost +local:root
docker run -it --net=host --privileged --rm  -e MASTER_IP=$1 -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix  -v /root/.Xauthority:/root/.Xauthority:rw  co4robots/co4robotsgui bash
#docker run  --name co4robotsGUI -i -t --net=host --rm -e DISPLAY=$DISPLAY:0 -e MASTER_IP=$1 -v /tmp/.X11-unix:/tmp/.X11-unix co4robots/co4robotsgui bash

