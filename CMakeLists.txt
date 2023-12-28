cmake_minimum_required(VERSION 3.0.2)
project(robotics_2_project)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  rospy
  catkin REQUIRED roslaunch
  std_msgs
  message_generation
)

add_message_files(
    FILES
    pose_message.msg
    path_message.msg	
)

generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )

roslaunch_add_file_check(launch)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS				
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(
  DIRECTORY launch scripts src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

