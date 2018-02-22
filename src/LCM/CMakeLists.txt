agile_include()

add_executable(unityImageServer unityImageServer.cpp)
agile_link_target(unityImageServer)
target_link_libraries(unityImageServer zmq zmqpp ${OpenCV_LIBS})

add_executable(unityPoseServer unityPoseServer.cpp)
agile_link_target(unityPoseServer)
target_link_libraries(unityPoseServer zmq zmqpp)
