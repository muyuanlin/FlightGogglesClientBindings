cmake_minimum_required(VERSION 3.0)
project(flightgoggles_client_bindings)

set(CMAKE_CXX_STANDARD 11)

# Fix cuda linking issues
set(CUDA_USE_STATIC_CUDA_RUNTIME, OFF)

# Find OpenCV
find_package( OpenCV REQUIRED )

# Find Eigen
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_subdirectory(src)


