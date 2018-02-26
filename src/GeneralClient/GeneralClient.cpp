/**
 * @file   GeneralClient.cpp
 * @author Winter Guerra
 * @brief  Pulls images from Unity and saves them as PNGs
 *space.
 **/

#include "GeneralClient.hpp"

#define SHOW_DEBUG_IMAGE_FEED false


///////////////////////
// Constructors
///////////////////////

GeneralClient::GeneralClient(){
}

void GeneralClient::populateRenderSettings(){
  // Scene/Render settings
  state.maxFramerate = 60; 
  state.sceneIsInternal = true;
  state.sceneFilename = "Butterfly_World";
  state.compressImage = false; // Deprecated. Will be removed in the future.
  
  // Frame Metadata
  state.camWidth = 1024;
  state.camHeight = 768;
  state.camFOV = 70.0f;
  state.camDepthScale = 0.01; // 5cm resolution
  
  // Prepopulate metadata of cameras
  unity_outgoing::Camera_t cam_RGB;
  cam_RGB.ID = "Camera_RGB";
  cam_RGB.channels = 3;
  cam_RGB.isDepth = false;
  cam_RGB.outputIndex = 0;

  unity_outgoing::Camera_t cam_D;
  cam_D.ID = "Camera_D";
  cam_D.channels = 1;
  cam_D.isDepth = true;
  cam_D.outputIndex = 1;

  // Add cameras to persistent state
  state.cameras.push_back(cam_RGB);
  state.cameras.push_back(cam_D);
  
}

void GeneralClient::setCameraPoseUsingROSCoordinates(Eigen::Affine3d ros_pose, int cam_index) {
  // To transforms
  Transform3 NED_pose = convertENUToNEDCoordinates(ros_pose);
  Transform3 unity_pose = convertNEDGlobalPoseToGlobalUnityCoordinates(NED_pose);

  // Extract position and rotation
  std::vector<double> position = {
    unity_pose.translation()[0],
    unity_pose.translation()[1],
    unity_pose.translation()[2],
  };

  Eigen::Matrix3d rotationMatrix = unity_pose.rotation();
  Quaternionx quat(rotationMatrix);

  std::vector<double> rotation = {
    quat.x(),
    quat.y(),
    quat.z(),
    quat.w(),
  };

  // Set camera position and rotation
  state.cameras[cam_index].position = position;
  state.cameras[cam_index].rotation = rotation;
}

////////////////////////////////////
// Example consumers and publishers
////////////////////////////////////

void imageConsumer(GeneralClient *self){
    while (true){
      // Wait for render result (blocking).
      unity_incoming::RenderOutput_t renderOutput = self->flightGoggles.handleImageResponse();

      // Display result
      if (SHOW_DEBUG_IMAGE_FEED){
        cv::imshow("Debug RGB", renderOutput.images[0]);
        cv::imshow("Debug D", renderOutput.images[1]);
        cv::waitKey(1);
      }
    }
}

void posePublisher(GeneralClient *self){
  // Sends render requests to FlightGoggles indefinitely
  while (true){
    // Update timestamp of state message (needed to force FlightGoggles to rerender scene)
    self->state.utime = self->flightGoggles.getTimestamp();
    // request render
    self->flightGoggles.requestRender(self->state);
    // Throttle requests to framerate.
    usleep(1e6/self->state.maxFramerate);
    }
}

///////////////////////
// Example Client Node
///////////////////////

int main(int argc, char **argv) {
  // Create client
  GeneralClient generalClient;

  // Load params
  generalClient.populateRenderSettings();

  // Prepopulate FlightGoggles state with camera pose
  Transform3 camera_pose;
  camera_pose.translation() = Vector3(0,-1,1);
  // Set rotation matrix using pitch, roll, yaw
  camera_pose.linear() = Eigen::AngleAxisd(M_PI/4.0f, Eigen::Vector3d(0,0,0)).toRotationMatrix();

  // Populate status message with new pose
  generalClient.setCameraPoseUsingROSCoordinates(camera_pose, 0);
  generalClient.setCameraPoseUsingROSCoordinates(camera_pose, 1);

  // Fork sample render request thread
  std::thread posePublisherThread(posePublisher, &generalClient);

  // Fork a sample image consumer thread
  std::thread imageConsumerThread(imageConsumer, &generalClient);

  // Spin
  while (true) {sleep(1);}

  return 0;
}
