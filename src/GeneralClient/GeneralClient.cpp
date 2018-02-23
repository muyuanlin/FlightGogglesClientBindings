/**
 * @file   GeneralClient.cpp
 * @author Winter Guerra
 * @brief  Pulls images from Unity and saves them as PNGs
 *space.
 **/

#include "GeneralClient.hpp"


///////////////////////
// Constructors
///////////////////////

GeneralClient::GeneralClient(){

}

void GeneralClient::populateRenderSettings(){
  // Scene/Render settings
  state.maxFramerate = 1e6; // make framerate really high
  state.sceneIsInternal = true;
  state.sceneFilename = "Butterfly_world";
  state.compressImage = false; // Deprecated. Will be removed in the future.
  
  // Frame Metadata
  state.camWidth = 1024;
  state.camHeight = 768;
  state.camFOV = 70.0f;
  state.camDepthScale = 0.05; // 5cm resolution
  
  // Prepopulate metadata of cameras
  unity_outgoing::Camera_t cam_RGB;
  cam_RGB.ID = "Camera_RGB";
  cam_RGB.channels = 3;
  cam_RGB.isDepth = false;
  cam_RGB.outputIndex = 0;

  unity_outgoing::Camera_t cam_D;
  cam_RGB.ID = "Camera_D";
  cam_RGB.channels = 1;
  cam_RGB.isDepth = true;
  cam_RGB.outputIndex = 1;

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



int main(int argc, char **argv) {
  // Create class
  GeneralClient generalClient;

  // Load params
  generalClient.populateRenderSettings();

  // To test, output camera poses 
  while(true) {

    // Generate a camera pose
    Transform3 camera_pose;
    camera_pose.translation() = Vector3(0,0,1.0);
    camera_pose.rotate(Eigen::AngleAxisd(M_PI/4.0f, Eigen::Vector3d(0,0,1)));

    // Populate status message with new pose
    generalClient.setCameraPoseUsingROSCoordinates(camera_pose, 0);
    generalClient.setCameraPoseUsingROSCoordinates(camera_pose, 1);
    
    // request render
    generalClient.flightGoggles.requestRender(generalClient.state);

    // Wait for render result.
    unity_incoming::RenderOutput_t renderOutput = generalClient.flightGoggles.handleImageResponse();

    // Display result
    cv::imshow("Debug RGB", renderOutput.images[0]);
    cv::imshow("Debug D", renderOutput.images[1]);

  }

  return 0;
}
