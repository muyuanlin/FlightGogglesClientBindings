#ifndef UNITYMESSAGESPEC_H
#define UNITYMESSAGESPEC_H
/**
 * @file   jsonMessageSpec.hpp
 * @author Winter Guerra
 * @brief  Defines the json structure of messages going to and from Unity.
 */

// Message/state struct definition

#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "json.hpp"
using json = nlohmann::json;

namespace unity_outgoing {

struct Camera_t {
  std::string ID;
  std::vector<double> position;
  std::vector<double> rotation;
  // Metadata
  int channels;
  bool isDepth;
  int outputIndex;
};

// Window class for decoding the ZMQ messages.
struct Window_t {
  std::string ID;
  std::vector<double> position;
  std::vector<double> rotation;
  // Metadata
  std::vector<double> color;
  std::vector<double> size;
};

struct StateMessage_t {
  // Constant values for startup
  int maxFramerate;
  bool sceneIsInternal;
  std::string sceneFilename;
  bool compressImage;
  // Frame Metadata
  int64_t utime;
  int camWidth;
  int camHeight;
  float camFOV;
  double camDepthScale;
  // Object state update
  std::vector<Camera_t> cameras;
  std::vector<Window_t> windows;
};

// Json constructors

// StateMessage_t
inline void to_json(json& j, const StateMessage_t& o) {
  j = json{// Initializers
           {"maxFramerate", o.maxFramerate},
           {"sceneIsInternal", o.sceneIsInternal},
           {"sceneFilename", o.sceneFilename},
           {"compressImage", o.compressImage},
           // Frame Metadata
           {"utime", o.utime},
           {"camWidth", o.camWidth},
           {"camHeight", o.camHeight},
           {"camFOV", o.camFOV},
           {"camDepthScale", o.camDepthScale},
           // Object state update
           {"cameras", o.cameras},
           {"windows", o.windows}};
}

// Camera_t
inline void to_json(json& j, const Camera_t& o) {
  j = json{{"ID", o.ID},
           {"position", o.position},
           {"rotation", o.rotation},
           {"channels", o.channels},
           {"isDepth", o.isDepth},
           {"outputIndex", o.outputIndex}};
}

// Window_t
inline void to_json(json& j, const Window_t& o) {
  j = json{{"ID", o.ID},
           {"position", o.position},
           {"rotation", o.rotation},
           {"color", o.color},
           {"size", o.size}};
}
}

// Struct for returning metadata from Unity.
namespace unity_incoming {

struct RenderMetadata_t {
  // Metadata
  int64_t utime;
  int camWidth;
  int camHeight;
  bool isCompressed;
  double camDepthScale;
  // Object state update
  std::vector<std::string> cameraIDs;
  std::vector<int> channels;
};

// Json Parsers

// RenderMetadata_t
inline void from_json(const json& j, RenderMetadata_t& o) {
  o.utime = j.at("utime").get<int64_t>();
  o.camWidth = j.at("camWidth").get<int>();
  o.camHeight = j.at("camHeight").get<int>();
  o.camDepthScale = j.at("camDepthScale").get<double>();
  o.isCompressed = j.at("isCompressed").get<bool>();
  o.cameraIDs = j.at("cameraIDs").get<std::vector<std::string>>();
  o.channels = j.at("channels").get<std::vector<int>>();
}

// Struct for outputting parsed received messages to handler functions
struct RenderOutput_t {
  RenderMetadata_t renderMetadata;
  std::vector<cv::Mat> images;
};

}

#endif