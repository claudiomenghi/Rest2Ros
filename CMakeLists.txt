cmake_minimum_required(VERSION 2.8.3)
project(ugot)

find_package(catkin REQUIRED 
					COMPONENTS 
					rospy 
					actionlib_msgs 
					actionlib)

catkin_python_setup()


catkin_package()

include_directories(
include/${PROJECT_NAME} ${catkin_INCLUDE_DIRS} 
)


