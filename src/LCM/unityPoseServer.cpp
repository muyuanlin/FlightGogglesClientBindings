/**
 * @file   unityPoseServer.cpp
 * @author Winter Guerra
 * @brief  Pod that outputs the expected pose of cameras based on motion capture
 * data to Unity for FlightGoggles operation.
 */

#include "unityPoseServer.hpp"

////////////////////////////////
// TRANSFORMS
////////////////////////////////

// This function converts right hand rule pose to left handed rule coordinates
// (as used by Unity).
Transform3 convertGlobalPoseToGlobalUnityCoordinates(
    Transform3 world_T_window, Transform3 unityWorld_T_world) {
  // Switch axis to unity axis.
  Matrix4 unity_pose_T_LCM_pose;
  // x->z, y->x, z->-y
  // clang-format off
  unity_pose_T_LCM_pose <<  0, 1, 0,  0,
                            0, 0, -1, 0,
                            1, 0, 0,  0,
                            0, 0, 0,  1;
  // clang-format on

  Transform3 unity_pose;

  unity_pose.matrix() = unity_pose_T_LCM_pose * unityWorld_T_world.matrix() *
                        world_T_window.matrix() *
                        unity_pose_T_LCM_pose.transpose();

  return unity_pose;
}

// Converts a given LCM drone pose and relative camera pose into a global Unity
// pose for the camera.
/* Move world_T_cam from drone (right handed) coordinate system to Unity (left
   handed) coordinate system:
   Input vector in unity coordinates goes through the following
   transformations:
   Unity coordinates -> Drone coordinates -> perform rotation -> unity
   coordinates
*/
Transform3 convertCameraAndDronePoseToUnityCoordinates(
    agile::state_t LCM_pose, Transform3 body_T_cam,
    Transform3 unityWorld_T_world) {
  Transform3 world_T_body;
  Quaternionx q;
  Vector3 v_p, v_s, v_pos_fixed;
  v_p =
      Vector3(LCM_pose.position[0], LCM_pose.position[1], LCM_pose.position[2]);
  v_s = Vector3(1, 1, 1);
  q = Quaternionx(LCM_pose.orient[0], LCM_pose.orient[1], LCM_pose.orient[2],
                  LCM_pose.orient[3]);

  // Make the transformation matrix
  world_T_body.fromPositionOrientationScale(v_p, q, v_s);

  Transform3 world_T_cam = world_T_body * body_T_cam;
  Matrix4 cam_T_unitycam;
  // clang-format off
  cam_T_unitycam << 0, 1, 0, 0,
                    0, 0, 1, 0,
                    1, 0, 0, 0,
                    0, 0, 0, 1;
  // clang-format on

  Transform3 world_T_unitycam;
  world_T_unitycam.matrix() =
      world_T_cam *
      cam_T_unitycam;  // Move from z aligned camera to x aligned camera

  // x->z, y->x, z->-y
  Matrix4
      unity_from_drone; /**< Coordinate change from drone coordinate system to
                           unity coordinate system */
  // clang-format off
  unity_from_drone << 0, 1,  0, 0,
                      0, 0, -1, 0,
                      1, 0,  0, 0,
                      0, 0,  0, 1;
  // clang-format on

  Transform3 unityWorld_T_unitycam_unity; /**< Transformation from camera to
                                             world in unity coordinates */
  unityWorld_T_unitycam_unity.matrix() =
      unity_from_drone * unityWorld_T_world.matrix() *
      world_T_unitycam.matrix() * unity_from_drone.transpose();

  return unityWorld_T_unitycam_unity;
}

// Helper function for converting state_t to Vector7
Vector7d state_tToVector7(const agile::state_t &pose) {
  Vector7d output;
  output << pose.position[0], pose.position[1], pose.position[2],
      pose.orient[0], pose.orient[1], pose.orient[2], pose.orient[3];
  return output;
}
// Helper function for converting Vector7 to state_t
agile::state_t vector7ToState_t(const Vector7d &vec) {
  agile::state_t pose;
  pose.position[0] = vec[0];
  pose.position[1] = vec[1];
  pose.position[2] = vec[2];

  pose.orient[0] = vec[3];
  pose.orient[1] = vec[4];
  pose.orient[2] = vec[5];
  pose.orient[3] = vec[6];

  return pose;
}

///////////////////////
// Constructor
///////////////////////

UnityPoseServer::UnityPoseServer() {
  // Make sure that LCM is good
  if (!lcm_.good()) {
    std::cerr << "Failed to open lcm" << std::endl;
    exit(0);
  }

  // Initialize pose buffer
  past_poses = new agile::DataBuffer<Vector7d>(1.0);

  // Load values from param server
  agile::ParamClient param;
  // ZMQ connection params
  unity_ip = param.getString("IPs.unity");
  my_ip = param.getString("IPs.sim");
  drone_ip = param.getString("IPs.drone");

  // pose server settings.
  max_framerate = param.getInt("Cameras.maxFramerate");
  pose_channel = param.getString("Unity.pose_channel");
  pose_port = param.getDouble("Unity.pose_port");
  param.tryGetBool("Unity.render_only_keyframes", render_only_keyframes);

  // Calculate and save unity Offset.
  Eigen::Vector4d unityRot4d = param.getEigen4d("Unity.initial_orient");
  Eigen::Vector3d unityTrans = param.getEigen3d("Unity.initial_translation");
  unityWorld_T_world_.fromPositionOrientationScale(
      unityTrans,
      Quaternionx(unityRot4d[0], unityRot4d[1], unityRot4d[2], unityRot4d[3]),
      Vector3(1, 1, 1));

  // Get the camera IDs
  camera_ids = param.getStringVector("Cameras.IDs");

  // Initialize static Unity state.
  unity_state = unity_outgoing::StateMessage_t();
  unity_state.camHeight = param.getInt("Cameras.height");
  unity_state.camWidth = param.getInt("Cameras.width");
  unity_state.camFOV = param.getDouble("Cameras.FOV");
  unity_state.camDepthScale = param.getDouble("Cameras.depthScale");
  unity_state.compressImage = param.getBool("Cameras.compressImage");
  unity_state.maxFramerate =
      max_framerate * 2;  // Allow unity to have a higher framerate.
  unity_state.sceneIsInternal = param.getBool("Unity.sceneIsInternal");
  unity_state.sceneFilename = param.getString("Unity.sceneFilename");

  // Loop through camera IDs and initialize the state and transforms for each
  for (uint i = 0; i < camera_ids.size(); i++) {
    // Create internal camera object for keeping track of state.
    CameraStruct camera;
    // Populate static parameters that will be passed to unity
    camera.unity.ID = camera_ids[i];
    camera.unity.channels =
        param.getInt(("Cameras." + camera.unity.ID + ".channels").c_str());
    camera.unity.isDepth =
        param.getBool(("Cameras." + camera.unity.ID + ".isDepth").c_str());
    camera.unity.outputIndex = i;

    // Get transformations from param server
    Eigen::Vector4d camRot4d =
        param.getEigen4d(("Cameras." + camera.unity.ID + ".quat").c_str());
    Eigen::Vector3d camTrans = param.getEigen3d(
        ("Cameras." + camera.unity.ID + ".translation").c_str());

    // Calculate Unity transformation
    Transform3 body_T_cam_;
    body_T_cam_.fromPositionOrientationScale(
        camTrans,
        Quaternionx(camRot4d[0], camRot4d[1], camRot4d[2], camRot4d[3]),
        Vector3(1, 1, 1));
    camera.body_T_cam = body_T_cam_;

    // save the camera into the internal object lookup table.
    cameras[camera.unity.ID] = camera;
    // debug
    std::cout << "Loaded transformations for camera ID: " << camera.unity.ID
              << std::endl;
  }

  // Check if we should output window poses.
  param.tryGetBool("Experiment.useWindow", use_window);

  // Get the window IDs
  if (use_window) {
    window_ids = param.getStringVector("Window.IDs");

    // Get the transformation information for each ID
    for (uint i = 0; i < window_ids.size(); i++) {
      // Create camera object
      WindowStruct window;

      // Populate static properties for Unity
      window.unity.ID = window_ids[i];
      window.unity.color = param.getDoubleVector(
          ("Window." + window.unity.ID + ".color").c_str());
      window.unity.size = param.getDoubleVector(
          ("Window." + window.unity.ID + ".size").c_str());

      // Get transformations
      Eigen::Vector4d windowRot4d =
          param.getEigen4d(("Window." + window.unity.ID + ".quat").c_str());
      Eigen::Vector3d windowTrans = param.getEigen3d(
          ("Window." + window.unity.ID + ".translation").c_str());
      // Calculate Unity transformation
      Transform3 world_T_window_;
      world_T_window_.fromPositionOrientationScale(
          windowTrans, Quaternionx(windowRot4d[0], windowRot4d[1],
                                   windowRot4d[2], windowRot4d[3]),
          Vector3(1, 1, 1));
      window.world_T_window = world_T_window_;
      // save the window
      windows[window.unity.ID] = window;
      std::cout << "Loaded transformations for window ID: " << window.unity.ID
                << std::endl;
    }
  }

  // Now that we know what our cameras are, get the keyframe channel for the
  // primary camera.
  keyframe_channel = "KEYFRAME_" + camera_ids[0];

  std::cout << "Done reading from param server" << std::endl;

  std::cout << "Initialize ZMQ connection" << std::endl;

  // Tell where ZMQ should listen.
  std::ostringstream socket_descriptor;
  socket_descriptor << "tcp://*:" << pose_port;
  // initialize the ZMQ socket context
  context = new zmqpp::context();
  // create and bind a server socket
  server = new zmqpp::socket(*context, zmqpp::socket_type::publish);
  server->bind(socket_descriptor.str());
  std::cout << "Started TCP pose server!" << std::endl;

  // Subscribe to LCM channels
  lcm_.subscribe(pose_channel, &UnityPoseServer::handlePoseUpdate, this);
  lcm_.subscribe(keyframe_channel, &UnityPoseServer::handleKeyframeUpdate,
                 this);
}


///////////////////////
// LCM Message handlers
///////////////////////
void UnityPoseServer::handlePoseUpdate(const alcm::ReceiveBuffer *rbuf,
                                       const std::string &chan,
                                       const agile::state_t *msg) {
  if (!render_only_keyframes) {
    // Render all incoming poses.
    requestRender(*msg);
  } else {
    // Log the pose in a buffer so that it is ready for when the keyframe
    // arrives.
    past_poses->insert(msg->utime, state_tToVector7(*msg));
  }
}

void UnityPoseServer::handleKeyframeUpdate(const alcm::ReceiveBuffer *rbuf,
                                           const std::string &chan,
                                           const agile::features_t *msg) {
  if (!render_only_keyframes) {
    // Ignore keyframes
    return;
  } else {
    // Find the pose that this keyframe references.
    auto keyframePose = past_poses->iterator_closest(msg->utime, 0);
    // Check that such a pose exists.
    if (!past_poses->iterator_at_end(keyframePose)) {
      // Get the pose.
      agile::state_t pose = vector7ToState_t(keyframePose->second);
      pose.utime = keyframePose->first;
      requestRender(pose);
    }
  }
}

///////////////////////
// Render Functions
///////////////////////

/**
 * This function is called when a new pose has been received.
 * If the pose is good, asks Unity to render another frame by sending a ZMQ
 * message.
 *
 * Pose Message format for ZMQ
 * -- ZMQ Message start
 * Topic String ("Pose")
 * JSON Status Update
 * -- ZMQ Message end.
 */
void UnityPoseServer::requestRender(const agile::state_t &pose) {
  // Make sure that we have a pose that is newer than the last rendered pose.
  if (!(pose.utime > unity_state.utime)) {
    // Skip this render frame.
    return;
  }

  // // Limit Unity framerate by throttling requests
  // if (pose.utime < (unity_state.utime + (1e6)/max_framerate)) {
  //   // Skip this render frame.
  //   return;
  // }
  // // Debug
  // std::cout << "Frame" << std::to_string(pose.utime) << std::endl;

  // Create new message object
  zmqpp::message msg;
  // Add topic header
  msg << "Pose";

  // Update timestamp
  unity_state.utime = pose.utime;

  // recalculate object positions and status
  updateUnityObjectPositions(pose);

  // Create JSON object for status update & append to message.
  json json_msg = unity_state;
  msg << json_msg.dump();

  // Output debug messages at 1hz
  if (pose.utime > utime_of_last_debug + 1e6) {
    std::cout << "Last message sent: \"";
    std::cout << msg.get<std::string>(0) << std::endl;
    // Print JSON object
    std::cout << json_msg.dump(4) << std::endl;
    std::cout << "===================" << std::endl;
    // reset time of last debug message
    utime_of_last_debug = pose.utime;
  }
  // Send message without blocking.
  server->send(msg, TRUE);
}

// Iterates through all camera and window objects and calls an update function.
// Packs the resulting unity status structs into the umbrella unity_status
// struct.
void UnityPoseServer::updateUnityObjectPositions(const agile::state_t &pose) {
  // Clear old unity object data.
  unity_state.cameras = {};
  unity_state.windows = {};

  // Update cameras
  for (uint i = 0; i < camera_ids.size(); i++) {
    std::string ID = camera_ids[i];
    CameraStruct camera = cameras[ID];
    updateCameraPosition(camera, pose);
    unity_state.cameras.push_back(camera.unity);
  }
  if (use_window) {
    // Update Windows
    for (uint i = 0; i < window_ids.size(); i++) {
      std::string ID = window_ids[i];
      WindowStruct window = windows[ID];
      updateWindowPosition(window);
      unity_state.windows.push_back(window.unity);
    }
  }
}

// Helper function that calculates transforms for a camera, then outputs a
// struct that can be converted to JSON.
void UnityPoseServer::updateCameraPosition(CameraStruct &camera,
                                           const agile::state_t &pose) {
  // Convert camera pose to unity coordinates
  Transform3 unityCoordinates = convertCameraAndDronePoseToUnityCoordinates(
      pose, camera.body_T_cam, unityWorld_T_world_);
  Vector3 pos = unityCoordinates.translation();
  Quaternionx oriUnity = Quaternionx(unityCoordinates.rotation());

  // Update unity representation of position rotation state.
  camera.unity.position = std::vector<double>({pos[0], pos[1], pos[2]});
  camera.unity.rotation = std::vector<double>(
      {oriUnity.x(), oriUnity.y(), oriUnity.z(), oriUnity.w()});
}

// Helper function that calculates transforms for a window, then outputs a
// struct that can be converted to JSON.
void UnityPoseServer::updateWindowPosition(WindowStruct &window) {
  // Convert pose to unity coordinates
  Transform3 unityCoordinates = convertGlobalPoseToGlobalUnityCoordinates(
      window.world_T_window, unityWorld_T_world_);
  Vector3 pos = unityCoordinates.translation();
  Quaternionx oriUnity = Quaternionx(unityCoordinates.rotation());

  // Update unity representation of position rotation state.
  window.unity.position = std::vector<double>({pos[0], pos[1], pos[2]});
  window.unity.rotation = std::vector<double>(
      {oriUnity.x(), oriUnity.y(), oriUnity.z(), oriUnity.w()});
}


int main(int argc, char **argv) {
  // Create pose server
  UnityPoseServer unityPoseServer;
  // Handle messages
  while (true) {
    unityPoseServer.lcm_.handle();
  }

  return 0;
}
