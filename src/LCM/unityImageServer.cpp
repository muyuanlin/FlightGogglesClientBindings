/**
 * @file   unityImageServer.cpp
 * @author Winter Guerra
 * @brief  Pulls images from Unity and saves them as PNGs
 *space.
 **/

#include "unityImageServer.hpp"

namespace agile {

using namespace std;

///////////////////////
// Constructors
///////////////////////

UnityImageServer::UnityImageServer() {
  // Make sure that LCM is good
  if (!lcm_.good()) {
    std::cerr << "Failed to open lcm" << std::endl;
    exit(0);
  }
  // Get params
  getParams();
  // Initialize LCM callbacks
}


void UnityImageServer::getParams() {
  // Load values from param server
  agile::ParamClient param;
  // Connection params
  unity_ip = param.getString("IPs.unity");
  my_ip = param.getString("IPs.sim");
  drone_ip = param.getString("IPs.drone");
  image_port = param.getDouble("Unity.image_port");
}

/**
   * @brief Thread function for ZMQ Image PULL subscriber.
   *
   * Is on a seperate thread, sits in an infinite loop waiting for images.
   *
   * Image message format:
   * ----
   * camera_id
   * timestamp
   * image_data
   */
void imageClient(UnityImageServer *unityImageServer) {
  // Recast unityImageServer
  UnityImageServer *self = (UnityImageServer *)unityImageServer;

  // Tell where ZMQ should listen.
  std::ostringstream socket_descriptor;
  socket_descriptor << "tcp://" << self->my_ip << ":" << self->image_port;

  // initialize the ZMQ socket context
  zmqpp::context context;

  // Print debug info.
  std::cout << socket_descriptor.str() << std::endl;

  // create, configure, and bind a server socket
  zmqpp::socket client(context, zmqpp::socket_type::subscribe);
  // client.set(zmqpp::socket_option::receive_high_water_mark, 100);
  client.bind(socket_descriptor.str());
  client.subscribe("");
  std::cout << "Starting TCP image subscriber!" << std::endl;

  /* ZMQ messages have the following format
    * JSON metadata
    * N images, each in its own frame.
    */

  int rate_limiter = 0;
  tstamp u_latency = 0;
  tstamp last_debug_message_timestamp = getTimestamp();
  // Only output debug messages at 1hz
  uint8_t num_frames = 100;
  while (true) {
    // Get data from client as fast as possible
    zmqpp::message msg;
    client.receive(msg);

    // Sanity check the packet.
    if (msg.parts() > 1) {
      // Unpack message metadata.
      std::string json_metadata_string = msg.get(0);
      // Parse metadata.
      unity_incoming::RenderMetadata_t renderMetadata =
          json::parse(json_metadata_string);

      // Log the latency in ms (1,000 microseconds)
      if (!u_latency) {
        u_latency = (getTimestamp() - renderMetadata.utime);
      } else {
        // avg over last 10 frames
        u_latency =
            ((u_latency * (9) + (getTimestamp() - renderMetadata.utime)) / 10);
      }
      rate_limiter++;

      // Make sure that input buffer is allocated
      if (!self->is_buffer_initialized) {
        // Allocate buffer for use in typecasting of input images
        self->_castedInputBuffer.resize(renderMetadata.camWidth *
                                        renderMetadata.camHeight * 3);
        self->is_buffer_initialized = true;
      }

      // For each camera, save the received image.
      for (uint i = 0; i < renderMetadata.cameraIDs.size(); i++) {
        // Reshape the received image
        // Calculate how long the casted and reshaped image will be.
        uint32_t imageLen = renderMetadata.camWidth * renderMetadata.camHeight *
                            renderMetadata.channels[i];
        // Get raw image string from ZMQ message
        std::string imageData = msg.get(i + 1);
        // ALL images comes as 3-channel images from Unity. However, if this
        // camera is supposed to be single channel, we need to discard the other
        // 2 channels.
        // Note: We assume that if the camera is supposed to be single channel,
        // Unity has already done the "Right Thing (TM)" and done grayscale
        // conversion for us.
        // This is necessary since Unity is optimized for outputting 3-channel
        // images and dropping channels in Unity would be costly.
        uint32_t bufferRowLength =
            renderMetadata.camWidth * renderMetadata.channels[i];
        // Convert image data into std::vector<uint8_t> by iterating over the
        // output domain
        for (uint16_t y = 0; y < renderMetadata.camHeight; y++) {
          // Additionally, the images that come from Unity are Y-inverted, so we
          // should invert the rows.
          uint16_t inv_y = renderMetadata.camHeight - y - 1;
          for (uint16_t x = 0; x < renderMetadata.camWidth; x++) {
            for (uint8_t c = 0; c < renderMetadata.channels[i]; c++) {
              // cast the input
              self->_castedInputBuffer[y * bufferRowLength +
                                       x * renderMetadata.channels[i] + c] =
                  imageData[inv_y * renderMetadata.camWidth * 3 + x * 3 + c];
            }
          }
        }

        // Construct the output filename
        std::string output_filename =
            "../../logs/unity_images/" + renderMetadata.cameraIDs[i] + "_" +
            std::to_string(renderMetadata.utime) + ".ppm";
        // Save as ppm
        // Choose to output color or gray
        std::string ppm_type;
        if (renderMetadata.channels[i] == 1) {
          ppm_type = "P5";
        } else {
          ppm_type = "P6";
        }
        // Open file
        ofstream f;
        f.open(output_filename, ios::out | ios::binary);
        if (f.is_open()) {
          // Output PPM metadata
          f << ppm_type << "\n\n";
          f << renderMetadata.camWidth << " " << renderMetadata.camHeight
            << "\n";
          f << "255\n";
          // Output image data
          f.write((char *)&(self->_castedInputBuffer[0]), imageLen);
          f.close();
        } else {
          std::cout << "unable to open " << output_filename << std::endl;
          exit(-1);
        }
      }

      if (rate_limiter >= num_frames) {
        // Log update FPS
        std::cout << "Update rate: "
                  << (num_frames * 1000000.0) /
                         (getTimestamp() - last_debug_message_timestamp)
                  << " ms_latency: " << u_latency / 1000 << std::endl;

        last_debug_message_timestamp = getTimestamp();

        rate_limiter = 0;
      }
    }
  }
}


}

using namespace agile;

int main(int argc, char **argv) {
  // Create class
  UnityImageServer unityImageServer;

  // Create an image client thread
  std::thread imageClientThread(imageClient, &unityImageServer);

  // Handle messages
  while (true) {
    unityImageServer.lcm_.handle();
  }
  return 0;
}
