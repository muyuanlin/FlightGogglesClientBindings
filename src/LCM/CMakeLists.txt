add_executable(LCMClient LCMClient.cpp)
target_link_libraries(LCMClient ${OpenCV_LIBS} FlightGogglesClient)
