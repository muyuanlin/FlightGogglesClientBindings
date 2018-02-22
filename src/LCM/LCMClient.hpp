#ifndef LCMCLIENT_H
#define LCMCLIENT_H
/**
 * @file   LCMClient.hpp
 * @author Winter Guerra
 * @brief  LCM client interface for FlightGoggles.
 */

#include <FlightGogglesClient.hpp>

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
