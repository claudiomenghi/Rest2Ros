#!/bin/bash

export ROS_HOSTNAME=$MASTER_IP
export ROS_MASTER_URI="http://$MASTER_IP"":11311"

ln /dev/null /dev/raw1394

service ssh restart
source /opt/ros/indigo/setup.bash
#catkin_make install
catkin build
source ./devel/setup.bash
export QT_X11_NO_MITSHM=1
#export LC_NUMERIC=c


#chromium-browser

##################################################################
# Running the packages of the co4robot project 
##################################################################





#gzserver



#roslaunch tiago_gazebo tiago_gazebo.launch public_sim:=true robot:=steel >> log.txt &
#roslaunch flexbe_onboard behavior_onboard.launch &
#roslaunch flexbe_widget behavior_ocs.launch &
screen -S ugotrest -dm sh -c 'sh ugot.sh; echo $?; exec bash -i; python rest_python.py'
#screen -S ugotrest1 -dm sh -c 'sh ugot.sh; echo $?; exec bash -i; python rest_python.py'
screen -S ugot -dm sh -c 'sh ugot.sh; echo $?; exec bash -i; java -jar specificationmanager.jar'




#screen -dmS kth
echo "co4robots platform welcome"
exec "$@"

