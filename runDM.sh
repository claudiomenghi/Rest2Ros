#!/bin/bash

if [ -z "$1" ]
  then
    echo "You should provide the ip of the master as second argument"
    exit 2
fi

xhost +local:root
docker run -it --net=host --privileged --rm  -e MASTER_IP=$1 -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix  -v /root/.Xauthority:/root/.Xauthority:rw  co4robots/co4robotsgui bash
#docker run  --name co4robotsGUI -i -t --net=host --rm -e DISPLAY=$DISPLAY:0 -e MASTER_IP=$1 -v /tmp/.X11-unix:/tmp/.X11-unix co4robots/co4robotsgui bash

#<<<<<<< HEAD
#xhost +
#MAC_IP=$(ifconfig en0 | grep inet | awk '$1=="inet" {print $2}')
#docker run  --name co4robotsGUI -i -t --rm -e  DISPLAY=$MAC_IP:0 -e MASTER_IP=$1 -p 13000:13000 -p 11311:11311    -v /tmp/.X11-unix:/tmp/.X11-unix co4robots/co4robotsgui bash
#=======
#>>>>>>> 76580ad91059d7d539e501349850ef77dbcd0cd0
