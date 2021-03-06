cmake_minimum_required(VERSION 2.8.3)
project(rmc_simulation)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs std_srvs geometry_msgs 
                                        tf tf2 dynamic_reconfigure message_generation
                                        genmsg)
include_directories(include ${catkin_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS actionlib_msgs actionlib)

find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

#add BEFORE catkin_package()
add_action_files(DIRECTORY action FILES PanServo.action)
generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package(
    DEPENDS gazebo
    CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs tf actionlib_msgs
    INCLUDE_DIRS include
    LIBRARIES
)


add_library(gazebo_ros_servo src/gazebo_ros_servo.cpp)
target_link_libraries(gazebo_ros_servo ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_dependencies(gazebo_ros_servo rmc_simulation_generate_messages_cpp)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
  PATTERN ".git" EXCLUDE)

install(DIRECTORY sdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.sdf"
  PATTERN ".git" EXCLUDE)
