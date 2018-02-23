/**
 * @file   unityPoseServer.cpp
 * @author Winter Guerra
 * @brief  Pod that outputs the expected pose of cameras based on motion capture
 * data to Unity for FlightGoggles operation.
 */

#include "FlightGogglesClient.hpp"

///////////////////////
// Constructor
///////////////////////

FlightGogglesClient::FlightGogglesClient()
{
}

void FlightGogglesClient::initializeConnections()
{
    std::cout << "Initializing ZMQ connections..." << std::endl;
    // // initialize the ZMQ socket context
    // context = new zmqpp::context();
    // create and bind a upload_socket
    upload_socket.bind(client_address+":"+upload_port);
    // create and bind a download_socket
    download_socket.bind(client_address+":"+download_port);
    std::cout << "Done!" << std::endl;
}

// @TODO
void FlightGogglesClient::ensureBufferIsAllocated(unity_incoming::RenderMetadata_t){

}


///////////////////////
// Render Functions
///////////////////////

/**
 * This function is called when a new pose has been received.
 * If the pose is good, asks Unity to render another frame by sending a ZMQ
 * message.
*/
bool FlightGogglesClient::requestRender(unity_outgoing::StateMessage_t requested_state)
{
    // Make sure that we have a pose that is newer than the last rendered pose.
    if (!(requested_state.utime > last_uploaded_utime))
    {
        // Skip this render frame.
        return false;
    }

    // // Limit Unity framerate by throttling requests
    // if (requested_state.utime < (last_uploaded_utime + (1e6)/max_framerate)) {
    //   // Skip this render frame.
    //   return;
    // }
    // Debug
    std::cout << "Frame " << std::to_string(requested_state.utime) << std::endl;

    // Create new message object
    zmqpp::message msg;
    // Add topic header
    msg << "Pose";

    // Update timestamp
    last_uploaded_utime = requested_state.utime;

    // Create JSON object for status update & append to message.
    json json_msg = requested_state;
    msg << json_msg.dump();

    // Output debug messages at 1hz
    if (requested_state.utime > last_debug_utime + 1e6)
    {
        std::cout << "Last message sent: \"";
        std::cout << msg.get<std::string>(0) << std::endl;
        // Print JSON object
        std::cout << json_msg.dump(4) << std::endl;
        std::cout << "===================" << std::endl;
        // reset time of last debug message
        last_debug_utime = requested_state.utime;
    }
    // Send message without blocking.
    upload_socket.send(msg, true);
    return true;
}

// @TODO
unity_incoming::RenderOutput_t handleImageResponse(){

}


