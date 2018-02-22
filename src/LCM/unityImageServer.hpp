#ifndef UNITYIMAGESERVER_H
#define UNITYIMAGESERVER_H
/**
 * @file   unityImageServer.hpp
 * @author Winter Guerra
 * @brief  Listens for incoming images from Unity
 */

#include <fstream>
// Include agile helpers for getting camera parameters
#include "cameraHandler/cameraParams.hpp"

// Include ZMQ bindings for comms with Unity.
#include <iostream>
#include <snappy.h>
#include <string>
#include <zmqpp/zmqpp.hpp>

// Include JSON message type definitions.
#include "jsonMessageSpec.hpp"
#include "agileFramework/json.hpp"
using json = nlohmann::json;

#include "agileFramework/agileFramework.hpp"
#include "agileFramework/paramsFramework.hpp"
#include <lcmtypes/bot_core.hpp>  // Image types

namespace agile {

class UnityImageServer {
 public:
   // temporary variables for LCM subscriptions
  alcm::LCM lcm_;

  // ZMQ connection parameters from the param server
  std::string my_ip;
  std::string unity_ip;
  std::string drone_ip;
  double image_port;

  std::string image_channel;
  // Camera parameters from param server
  int _numCameras = 0;
  std::vector<std::string> camera_ids;
  std::vector<CameraParams> _camParams;
  // Temporary buffer for reshaping and casting of images
  std::vector<uint8_t> _castedInputBuffer;
  bool is_buffer_initialized = false;

  UnityImageServer();

  void getParams();
};
}

#endif
