#ifndef FLIGHTGOGGLESCLIENT_H
#define FLIGHTGOGGLESCLIENT_H
/**
 * @file   FlightGogglesClient.hpp
 * @author Winter Guerra
 * @date   Feb 22, 2018
 * @brief  Library class that abstracts interactions with FlightGoggles.
 */

#include <fstream>

// Include ZMQ bindings for comms with Unity.
#include <iostream>
#include <string>
#include <zmqpp/zmqpp.hpp>

// Include JSON message type definitions.
#include "jsonMessageSpec.hpp"
#include "json.hpp"
using json = nlohmann::json;

// For transforms
#include <Eigen/Geometry>
#include <Eigen/Dense>

class FlightGogglesClient {
public:

  // ZMQ connection parameters
  std::string client_address = "tcp://*"; 
  std::string input_port = "10253";
  std::string output_port = "10254";
  // Temporary buffer for reshaping and casting of received images
  std::vector<uint8_t> _castedInputBuffer;
  bool is_buffer_initialized = false;

    // Methods
    FlightGogglesClient();

    // Send render request to Unity
    void requestRender(json status);

    RenderOutput_t handleImageResponse();    

};


#endif