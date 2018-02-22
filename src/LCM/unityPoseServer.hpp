#ifndef UNITYPOSESERVER_H
#define UNITYPOSESERVER_H
/**
 * @file   unityPoseServer.hpp
 * @author Winter Guerra
 * @brief  Pod that outputs the expected pose of cameras based on motion capture
 data to Unity for FlightGoggles operation.

 */

#include "agileFramework/agileFramework.hpp"
#include "agileFramework/eigenDef.hpp"
#include "agileFramework/paramsFramework.hpp"
// For transforms
#include <Eigen/Geometry>
#include <Eigen/Dense>
// For socketing with Unity
#include <iostream>
#include <string>
#include <zmqpp/zmqpp.hpp>
// Include JSON message type definitions.
#include "jsonMessageSpec.hpp"
#include "agileFramework/json.hpp"
using json = nlohmann::json;
// For keyframe synchronization
#include "agileFramework/data_buffer.hpp"
#include "agileFramework/eigenDef.hpp"

#define PRECISION 10
#define DONTALIGNCOLS 1

// define some eigen types and formats
typedef Eigen::Vector3d Vector3;
typedef Eigen::Quaternion<double> Quaternionx;
typedef Eigen::Affine3d Transform3;
typedef Eigen::Matrix4d Matrix4;
typedef Eigen::Matrix3d Matrix3;
Eigen::IOFormat CSV(PRECISION, DONTALIGNCOLS, ",", ",", "", "", "", "");

// Simple camera object for multicam operation.
struct CameraStruct {
  // For internal use
  Transform3 body_T_cam;
  // For use by unity
  unity_outgoing::Camera_t unity;
};

struct WindowStruct {
  // For internal use
  Transform3 world_T_window;
  // For use by Unity
  unity_outgoing::Window_t unity;
};

class UnityPoseServer {
 public:
  // temporary variables for LCM subscriptions
  alcm::LCM lcm_;

  // Transform3 body_T_cam_; /**< Transformation from the camera to the body */
  Transform3
      unityWorld_T_world_; /**< Transformation from world coordinates to unity
                              world coordinates */

  // ZMQ connection settings
  std::string my_ip;
  std::string unity_ip;
  std::string drone_ip;
  double pose_port;
  // ZMQ connection state vars (stored as pointers)
  zmqpp::context *context;
  zmqpp::socket *server;

  // LCM subscription channels
  std::string pose_channel;
  std::string keyframe_channel;

  // Vars for keeping track of keyframe poses.
  bool render_only_keyframes = false;
  agile::DataBuffer<Vector7d> *past_poses;
  agile::state_t last_received_pose;

  // Unity Settings
  uint8_t max_framerate;

  // State message to send to Unity.
  unity_outgoing::StateMessage_t unity_state;

  // Cameras
  std::vector<std::string> camera_ids;
  std::map<std::string, CameraStruct> cameras;

  // Windows
  bool use_window = false;
  std::vector<std::string> window_ids;
  std::map<std::string, WindowStruct> windows;

  // throttling vars for debug messages
  int64_t utime_of_last_debug = 0;

  // Constructor
  UnityPoseServer();

  // Message handlers
  void handlePoseUpdate(const alcm::ReceiveBuffer *rbuf,
                        const std::string &chan, const agile::state_t *msg);
  void handleKeyframeUpdate(const alcm::ReceiveBuffer *rbuf,
                            const std::string &chan,
                            const agile::features_t *msg);

  // Helper functions
  void requestRender(const agile::state_t &pose);
  void updateUnityObjectPositions(const agile::state_t &pose);
  void updateCameraPosition(CameraStruct &camera, const agile::state_t &pose);
  void updateWindowPosition(WindowStruct &window);
};

#endif
