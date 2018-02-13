#!/bin/bash

if [ -z "$1" ]
  then
    echo "You should provide the ip of the master as second argument"
    exit 2
fi


docker run  --name co4robotsGUI -i -t --net=host --rm -e DISPLAY=$DISPLAY:0 -e MASTER_IP=$1 -v /tmp/.X11-unix:/tmp/.X11-unix co4robots/co4robotsgui bash
