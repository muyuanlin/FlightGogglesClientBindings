cmake_minimum_required(VERSION 2.8.3)
project(flight_goggles_client)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-Ofast")

# Fix cuda linking issues
set(CUDA_USE_STATIC_CUDA_RUNTIME, OFF)

# Find OpenCV
find_package( OpenCV REQUIRED )

# Find Eigen
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# add_subdirectory(src)


include_directories(
    include
# ${catkin_INCLUDE_DIRS}
)

# Add library
add_library(flight_goggles_client STATIC src/flight_goggles_client.cpp src/flight_goggles_client.hpp)

# Link in needed libraries
target_link_libraries(flight_goggles_client zmq zmqpp ${OpenCV_LIBS})

# Expose as library
target_include_directories(flight_goggles_client PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})



