cmake_minimum_required(VERSION 2.8.3)
project(flight_goggles_client_example)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  tf2
  nav_msgs
  sensor_msgs
  geometry_msgs
)

find_package( Eigen3 REQUIRED )


catkin_package(
    LIBRARIES simulator
    CATKIN_DEPENDS roscpp rospy std_msgs
    # DEPENDS system_lib
    tf
    tf2
)


include_directories(
  src
  ../include/lib
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)



add_executable(synthetic_images_pulisher ./src/synthetic_images_pulisher.cpp)
target_link_libraries(synthetic_images_pulisher ${OpenCV_LIBS} flight_goggles_client -lpthread ${catkin_LIBRARIES})

add_executable(trajectory_generation ./src/trajectory_generation.cpp)
target_link_libraries(trajectory_generation ${catkin_LIBRARIES})





