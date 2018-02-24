#ifndef GENERALCLIENT_H
#define GENERALCLIENT_H
/**
 * @file   GeneralClient.hpp
 * @author Winter Guerra
 * @brief  Basic client interface for FlightGoggles.
 */

#include <FlightGogglesClient.hpp>
// #include <jsonMessageSpec.hpp>

#include <iostream>
#include <string>
#include <vector>
#include <thread>

// For converting ROS/LCM coordinates to Unity coordinates
#include "transforms.hpp"

class GeneralClient {
 public:
  // FlightGoggles interface object
  FlightGogglesClient flightGoggles;

  // Base status object (which holds camera settings, env settings, etc)
  unity_outgoing::StateMessage_t state;

  // constructor
  GeneralClient();

  // Populate starting settings into state
  void populateRenderSettings();

  void setCameraPoseUsingROSCoordinates(Eigen::Affine3d ros_pose, int cam_index);



};

#endif
