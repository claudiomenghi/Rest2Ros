#!/bin/bash

#<<<<<<< HEAD
#=======
LOCAL_IP=$(/sbin/ifconfig wlan0 | grep 'inet addr' | cut -d: -f2 | awk '{ print $1}')

export ROS_HOSTNAME=$LOCAL_IP
export ROS_MASTER_URI="http://$MASTER_IP"":11311"
#>>>>>>> 76580ad91059d7d539e501349850ef77dbcd0cd0

echo "export ROS_HOSTNAME=127.0.0.1">>~/.profile
echo "export ROS_MASTER_URI=http://$MASTER_IP:11311" >>~/.profile

. ~/.profile
export ROS_HOSTNAME=127.0.0.1
export ROS_MASTER_URI=http://10.68.0.1:11311

ln /dev/null /dev/raw1394

service ssh restart
source /opt/ros/indigo/setup.bash
#catkin_make install
catkin build
source ./devel/setup.bash
export QT_X11_NO_MITSHM=1
#export LC_NUMERIC=c

#if [ $MASTER_IP == "127.0.0.1" ]; then
roscore
#fi

#chromium-browser

##################################################################
# Running the packages of the co4robot project 
##################################################################





#gzserver



#roslaunch tiago_gazebo tiago_gazebo.launch public_sim:=true robot:=steel >> log.txt &
#roslaunch flexbe_onboard behavior_onboard.launch &
#roslaunch flexbe_widget behavior_ocs.launch &
screen -S ugotrest -dm sh -c 'sh ugotrest.sh;  exec bash -i;'
#screen -S ugotrest1 -dm sh -c 'sh ugot.sh; echo $?; exec bash -i; python rest_python.py'

#IP=$(ifconfig eth0 | grep "inet addr" | cut -d ':' -f 2 | cut -d ' ' -f 1)
screen -S ugotmissionspec -dm sh -c 'sh ugot.sh;  exec bash -i;'

#screen -S ugotmissionspec -dm sh -c 'sh ugot.sh; echo $?; java -jar specificationmanager.jar; exec bash -i;'



#screen -dmS kth
echo "co4robots platform welcome"
exec "$@"

