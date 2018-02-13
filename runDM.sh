#!/bin/bash

if [ -z "$1" ]
  then
    echo "You should provide the ip of the master as second argument"
    exit 2
fi


<<<<<<< HEAD
docker run  --name co4robotsGUI -i -t --net=host --rm -e DISPLAY=$DISPLAY:0 -e MASTER_IP=$1 -v /tmp/.X11-unix:/tmp/.X11-unix co4robots/co4robotsgui bash
=======
xhost +
MAC_IP=$(ifconfig en0 | grep inet | awk '$1=="inet" {print $2}')
docker run  --name co4robotsGUI -i -t --rm --net=host -e  DISPLAY=$MAC_IP:0 -e MASTER_IP=$1 -p 13000:13000   -v /tmp/.X11-unix:/tmp/.X11-unix co4robots/co4robotsgui bash
>>>>>>> f40a2d6517258375b0b89c002d1300cd21485c0a
