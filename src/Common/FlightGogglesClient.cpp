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
    initializeConnections();
}

void FlightGogglesClient::initializeConnections()
{
    std::cout << "Initializing ZMQ connections..." << std::endl;
    // // initialize the ZMQ socket context
    // context = new zmqpp::context();
    // create and bind a upload_socket
    upload_socket.bind(client_address + ":" + upload_port);
    // create and bind a download_socket
    download_socket.bind(client_address + ":" + download_port);
    download_socket.subscribe("");
    std::cout << "Done!" << std::endl;
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

    // Limit Unity framerate by throttling requests
    if (requested_state.utime < (last_uploaded_utime + (1e6)/requested_state.maxFramerate)) {
      // Skip this render frame.
      return false;
    }

    // Tell unity to always try its hardest to render as fast as it can.
    requested_state.maxFramerate = 1e3;

    // Debug
    // std::cout << "Frame " << std::to_string(requested_state.utime) << std::endl;

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
    if (requested_state.utime > last_upload_debug_utime + 1e6)
    {
        std::cout << "Last message sent: \"";
        std::cout << msg.get<std::string>(0) << std::endl;
        // Print JSON object
        std::cout << json_msg.dump(4) << std::endl;
        std::cout << "===================" << std::endl;
        // reset time of last debug message
        last_upload_debug_utime = requested_state.utime;
    }
    // Send message without blocking.
    upload_socket.send(msg, true);
    return true;
}

// This is a blocking call.
unity_incoming::RenderOutput_t FlightGogglesClient::handleImageResponse()
{
    // Populate output
    unity_incoming::RenderOutput_t output;

    // Get data from client as fast as possible
    zmqpp::message msg;
    download_socket.receive(msg);

    // Sanity check the packet.
    // if (msg.parts() <= 1)
    // {
    //     return NULL;
    // }
    // Unpack message metadata.
    std::string json_metadata_string = msg.get(0);
    // Parse metadata.
    unity_incoming::RenderMetadata_t renderMetadata = json::parse(json_metadata_string);

    // Log the latency in ms (1,000 microseconds)
    if (!u_packet_latency)
    {
        u_packet_latency = (getTimestamp() - renderMetadata.utime);
    }
    else
    {
        // avg over last ~10 frames
        u_packet_latency =
            ((u_packet_latency * (9) + (getTimestamp() - renderMetadata.utime)) / 10);
    }

    ensureBufferIsAllocated(renderMetadata);

    // For each camera, save the received image.
    for (uint i = 0; i < renderMetadata.cameraIDs.size(); i++)
    {

        // Reshape the received image
        // Calculate how long the casted and reshaped image will be.
        uint32_t imageLen = renderMetadata.camWidth * renderMetadata.camHeight * renderMetadata.channels[i];
        // Get raw image string from ZMQ message
        std::string imageData = msg.get(i + 1);
        // ALL images comes as 3-channel images from Unity. However, if this camera is supposed to be single channel, we need to discard the other 2 channels.
        // Note: We assume that if the camera is supposed to be single channel, Unity has already done the "Right Thing (TM)" and done grayscale conversion for us.
        // This is necessary since Unity is optimized for outputting 3-channel images and dropping channels in Unity would be costly.
        uint32_t bufferRowLength = renderMetadata.camWidth * renderMetadata.channels[i];
        // Convert image data into std::vector<uint8_t> by iterating over the output domain
        for (uint16_t y = 0; y < renderMetadata.camHeight; y++)
        {
            // Additionally, the images that come from Unity are Y-inverted, so we should invert the rows.
            uint16_t inv_y = renderMetadata.camHeight - y - 1;
            for (uint16_t x = 0; x < renderMetadata.camWidth; x++)
            {
                for (uint8_t c = 0; c < renderMetadata.channels[i]; c++)
                {
                    // cast the input
                    _castedInputBuffer[y * bufferRowLength + x * renderMetadata.channels[i] + c] = imageData[inv_y * renderMetadata.camWidth * 3 + x * 3 + c];
                }
            }
        }

        // 

        // Pack image into cv::Mat
        cv::Mat new_image = cv::Mat(renderMetadata.camHeight, renderMetadata.camWidth, CV_MAKETYPE(CV_8U, renderMetadata.channels[i]));
        memcpy(new_image.data, _castedInputBuffer.data(), imageLen);

        // debug
        // cv::imshow("Debug", new_image);

        // Add image to output vector
        output.images.push_back(new_image);
    }

    // Add metadata to output
    output.renderMetadata = renderMetadata;

    // Output debug at 1hz
    if (getTimestamp() > last_download_debug_utime + 1e6)
    {
        // Log update FPS
        std::cout << "Update rate: "
                  << (num_frames * 1e6) /
                         (getTimestamp() - last_download_debug_utime)
                  << " ms_latency: " << u_packet_latency / 1e3 << std::endl;

        last_download_debug_utime = getTimestamp();
        num_frames = 0;
    }
    num_frames++;

    return output;
}
