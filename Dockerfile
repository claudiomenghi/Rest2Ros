FROM ubuntu:14.04

COPY ./entrypoint.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/entrypoint.sh
ENTRYPOINT ["entrypoint.sh"]

ENV DEBIAN_FRONTEND noninteractive

RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# ----------------------------------------
# Miscellaneous
# ----------------------------------------
RUN apt-get update && \
      apt-get -y install 
      

# install git
RUN apt-get update && apt-get install -y git
RUN apt-get install -y build-essential
RUN apt-get install wget gedit --assume-yes
RUN apt-get install xorg openbox --assume-yes

RUN apt-get install software-properties-common --assume-yes
RUN add-apt-repository --yes ppa:xqms/opencv-nonfree
RUN  apt-get update
#RUN apt-get install libopencv-nonfree-dev --assume-yes
RUN mkdir -p /catkin_ws/src

# ----------------------------------------
# Installing ROS
# ----------------------------------------

RUN  sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'


RUN  apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

RUN  apt-get update

RUN  apt-get install ros-indigo-desktop-full  --assume-yes


RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws/
RUN rosdep init
RUN rosdep update


RUN apt-get install git python-rosinstall ros-indigo-desktop-full python-catkin-tools ros-indigo-joint-state-controller ros-indigo-twist-mux ros-indigo-ompl ros-indigo-controller-manager ros-indigo-moveit-core ros-indigo-moveit-ros-perception ros-indigo-moveit-ros-move-group ros-indigo-moveit-kinematics ros-indigo-moveit-ros-planning-interface ros-indigo-moveit-simple-controller-manager ros-indigo-moveit-planners-ompl ros-indigo-joy ros-indigo-joy-teleop ros-indigo-teleop-tools ros-indigo-control-toolbox ros-indigo-sound-play ros-indigo-navigation ros-indigo-eband-local-planner ros-indigo-depthimage-to-laserscan  ros-indigo-openslam-gmapping ros-indigo-gmapping ros-indigo-moveit-commander ros-indigo-geometry-experimental vim screen --assume-yes

WORKDIR ../
RUN mkdir -p /.gazebo/models/
WORKDIR /.gazebo/models/
#RUN wget  -r -R "index\.html*" http://models.gazebosim.org/
ADD . /
#RUN cp -r models.gazebosim.org/* ./


# ----------------------------------------
# Add here additional linux packages required 
# ---------------------------------------- 
RUN apt-get install vim --assume-yes

WORKDIR /catkin_ws/
# ----------------------------------------
# Installing the co4robot packages
# ----------------------------------------

RUN wget https://raw.githubusercontent.com/pal-robotics/tiago_tutorials/master/tiago_public.rosinstall


RUN rosinstall src /opt/ros/indigo tiago_public.rosinstall
RUN rm /etc/ros/rosdep/sources.list.d/20-default.list

RUN rosdep init
RUN rosdep update

RUN ["/bin/bash", "-c", "source /catkin_ws/src/setup.bash;  rosdep fix-permissions; rosdep update;"]

RUN apt-get install ros-indigo-sick-tim ros-indigo-humanoid-nav-msgs ros-indigo-hokuyo-node ros-indigo-moveit-ros-visualization  --assume-yes

RUN rosdep install --from-paths src --ignore-src --rosdistro indigo --skip-keys="opencv2 opencv2-nonfree pal_laser_filters speed_limit sensor_to_cloud" 


RUN echo 'debconf debconf/frontend select Dialog' | debconf-set-selections

#RUN wget https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb

#apt-get -f install

#RUN dpkg -i  --force-all google-chrome-stable_current_amd64.deb
#RUN apt-get install chromium-browser --assume-yes

WORKDIR /catkin_ws/src
#RUN env GIT_SSL_NO_VERIFY=true git clone https://claudiomenghi:co4robots@gitc4r.pal-robotics.com:8060/philippschillinger/simulation.git

#RUN env GIT_SSL_NO_VERIFY=true git clone https://claudiomenghi:co4robots@gitc4r.pal-robotics.com:8060/logothm/ms1_msgs.git

#RUN env GIT_SSL_NO_VERIFY=true git clone https://github.com/team-vigir/flexbe_behavior_engine.git

# installing the planner
#RUN env GIT_SSL_NO_VERIFY=true git clone https://claudiomenghi:co4robots@gitc4r.pal-robotics.com:8060/mengguo/ms1_kth.git


WORKDIR ../
RUN mkdir logs
COPY target/spec-0.0.1-jar-with-dependencies.jar specificationmanager.jar
COPY ugot.sh ugot.sh
COPY ugotrest.sh ugotrest.sh
COPY rest_python.py rest_python.py


RUN add-apt-repository --yes ppa:webupd8team/java
RUN apt-get update
RUN echo debconf shared/accepted-oracle-license-v1-1 select true |  sudo debconf-set-selections
RUN echo debconf shared/accepted-oracle-license-v1-1 seen true |  sudo debconf-set-selections
RUN apt-get install oracle-java8-installer --assume-yes


CMD    /usr/sbin/sshd -D


EXPOSE 13000
EXPOSE 22
EXPOSE 11311

RUN apt-get install openssh-server telnet --assume-yes

        
    
