set -e


#xhost +local:root

unameOut="$(uname -s)"
#case "${unameOut}" in
    #Linux*)     
#docker run -it --net=host --privileged --rm  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix  -v /root/.Xauthority:/root/.Xauthority:rw  co4robots/co4robots bash;;
    
    #Darwin*)    
    #if ! which 'xquartz' &>/dev/null; 
    #    then wget https://dl.bintray.com/xquartz/downloads/XQuartz-2.7.10.dmg
    #fi
    
    #export MAC_IP=$(ifconfig en0 | grep inet | awk '$1=="inet" {print $2}')
    #xhost +local:root
    #xhost + $MAC_IP
    #echo "Socat"
#   # socat TCP-LISTEN:6000,reuseaddr,fork UNIX-CLIENT:\"$DISPLAY\"
    
    #brew tap cartr/qt4
    #brew tap-pin cartr/qt4
    #brew install qt@4
    #brew tap osrf/simulation
    #xcode-select --install
    
#brew tap brewsci/science
#brew install sga
#    brew install ffmpeg
#    cmake -DCMAKE_C_COMPILER=/usr/bin/gcc -DCMAKE_CXX_COMPILER=/usr/bin/g++
#    cmake -DCMAKE_C_COMPILER=/usr/bin/gcc -DCMAKE_CXX_COMPILER=/usr/bin/g++
#    brew install gazebo2
#    brew install gazebo2 --with-bullet --with-simbody
    
#    cmake  -DCMAKE_C_FLAGS_RELEASE=-DNDEBUG -DCMAKE_CXX_FLAGS_RELEASE=-DNDEBUG - -DCMAKE_BUILD_TYPE=Release -DCMAKE_FIND_FRAMEWORK=LAST -DCMAKE_VERBOSE_MAKEFILE=ON -Wno-dev -DENABLE_TESTS_COMPILATION:BOOL=False -DFORCE_GRAPHIC_TESTS_COMPILATION:BOOL=True -DDARTCore_FOUND:BOOL=False   
    xhost +
    MAC_IP=$(ifconfig en0 | grep inet | awk '$1=="inet" {print $2}')
   #MAC_IP
    docker run  --name co4robotsGUI -i -t --rm -e DISPLAY=$MAC_IP:0 -e MASTER_IP=$1 -p 13000:13000   -v /tmp/.X11-unix:/tmp/.X11-unix co4robots/co4robotsgui bash

    #kill -9 $(lsof -ti :6000)
    
    
    
    # docker run -it --privileged --rm  -e DISPLAY=$MAC_IP:0 -v /tmp/.X11-unix:/tmp/.X11-unix   co4robots/co4robots bash
    
 
 #    ;;
     #docker run -d --name firefox -e DISPLAY=$ip:0 -v /tmp/.X11-unix:/tmp/.X11-unix jess/firefox
     #-v /root/.Xauthority:/root/.Xauthority:rw
    
#    *)          echo "Unknown operating system " 
#    ;;
#esac


#docker run -it -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY --net=host  --device /dev/snd co4robots/co4robots bash

