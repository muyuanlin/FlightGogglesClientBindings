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

// For converting ROS/LCM coordinates to Unity
#include "transforms.hpp"

// For image operations
#include <opencv2/opencv.hpp>

class FlightGogglesClient
{
  public:
    // ZMQ connection parameters
    std::string client_address = "tcp://*";
    std::string upload_port = "10253";
    std::string download_port = "10254";
    // Socket variables
    zmqpp::context context;
    zmqpp::socket upload_socket {
        context, 
        zmqpp::socket_type::publish};
    zmqpp::socket download_socket {
        context, 
        zmqpp::socket_type::subscribe};

    // Temporary buffer for reshaping and casting of received images
    std::vector<uint8_t> _castedInputBuffer;

    // Keep track of time of last sent/received messages
    int64_t last_uploaded_utime = 0;
    int64_t last_downloaded_utime = 0;
    int64_t last_debug_utime = 0;

    // Constructor.
    FlightGogglesClient();

    // Connects to FlightGoggles.
    void initializeConnections();

    // OUTPUT

    // Send render request to Unity
    bool requestRender(unity_outgoing::StateMessage_t requested_state);

    // INCOMING

    // Ensure that input buffer can handle the incoming message.
    void ensureBufferIsAllocated(unity_incoming::RenderMetadata_t);

    unity_incoming::RenderOutput_t handleImageResponse();
};

#endif