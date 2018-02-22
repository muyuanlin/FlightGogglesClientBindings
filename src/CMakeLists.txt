add_executable(LCMReceiver LCMReciever.cpp)
link_target(LCMReceiver)
target_link_libraries(LCMReceiver zmq zmqpp ${OpenCV_LIBS})

add_executable(unityPoseServer unityPoseServer.cpp)
agile_link_target(unityPoseServer)
target_link_libraries(unityPoseServer zmq zmqpp)
