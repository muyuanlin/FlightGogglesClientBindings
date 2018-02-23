#ifndef LCMCLIENT_H
#define LCMCLIENT_H
/**
 * @file   LCMClient.hpp
 * @author Winter Guerra
 * @brief  LCM client interface for FlightGoggles.
 */

#include <FlightGogglesClient.hpp>

#include <iostream>
#include <string>
#include <vector>
#include <lcm/lcm-cpp.hpp>

class LCMClient {
 public:
   // temporary variables for LCM subscriptions
  lcm::LCM lcm_;
  std::string image_output_channel;
  std::string pose_input_channel;

  LCMClient();

};

#endif
