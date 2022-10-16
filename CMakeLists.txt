cmake_minimum_required(VERSION 3.0.2)
project(vision_route_tracker)


find_package(catkin REQUIRED COMPONENTS
  rospy
)


catkin_package(
   CATKIN_DEPENDS rospy
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


