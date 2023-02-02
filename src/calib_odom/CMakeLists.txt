cmake_minimum_required(VERSION 3.0.2)
project(calib_odom)


add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  std_msgs
)

find_package(PCL REQUIRED QUIET)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${csm_INCLUDE_DIRS} 
)

link_directories(${csm_LIBRARY_DIRS})

add_executable(${PROJECT_NAME}_node
  src/main.cpp)

add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${csm_EXPORTED_TARGETS} 
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${csm_LIBRARIES}
)