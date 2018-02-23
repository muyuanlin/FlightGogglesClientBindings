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

void GeneralClient::setCameraPoseUsingROSCoordinates(Transform3 rosPose, int cam_index) {
  
}


int main(int argc, char **argv) {
  // Create class
  GeneralClient generalClient();

  // Load params
  generalClient.populateRenderSettings();

  // To test, output camera poses 
  while(TRUE) {

  }

  return 0;
}
