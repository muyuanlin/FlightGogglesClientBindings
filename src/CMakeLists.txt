# Add library
add_library(FlightGogglesClient STATIC FlightGogglesClient.cpp FlightGogglesClient.hpp)
# Link in needed libraries

target_link_libraries(FlightGogglesClient zmq zmqpp ${OpenCV_LIBS})
# Expose as library

target_include_directories(FlightGogglesClient PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
