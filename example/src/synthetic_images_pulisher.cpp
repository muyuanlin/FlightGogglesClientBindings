/**
 * @file   SyntheticImagesPulisher.cpp
 * @author Muyuan Lin, adapted from @Winter Guerra
 * @brief  Pulls images from Unity and publish them as ROS images.
 *
 **/

#include "synthetic_images_publisher.hpp"

#define SHOW_DEBUG_IMAGE_FEED false
typedef const boost::function< void(const nav_msgs::Odometry::ConstPtr &)> callback;

void SyntheticImagesPublisher::populateRenderSettings(){
  // Scene/Render settings
  state.maxFramerate = 60; 
  state.sceneIsInternal = true;
  /*
  sceneFilename = "Butterfly_World";
  sceneFilename = "FPS_Warehouse_Day";
  sceneFilename = "FPS_Warehouse_Night";
  sceneFilename = "Hazelwood_Loft_Full_Day";
  sceneFilename = "Hazelwood_Loft_Full_Night";
   */
  state.sceneFilename = "FPS_Warehouse_Day";
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

void SyntheticImagesPublisher::setCameraPoseUsingROSCoordinates(Eigen::Affine3d ros_pose, int cam_index) {
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


void SyntheticImagesPublisher::poseSubscriber(const nav_msgs::Odometry::ConstPtr& msg){
    // ROS_INFO("Seq: [%d]", msg->header.seq);
    // Sends render requests to FlightGoggles indefinitely
    geometry_msgs::Pose cam_pose_tf = msg->pose.pose;

    float x = cam_pose_tf.position.x;
    cam_pose_tf.position.x = -cam_pose_tf.position.y;
    cam_pose_tf.position.y = x;


    Transform3 cam_pose_eigen;
    poseMsgToEigen(cam_pose_tf, cam_pose_eigen);

    // Populate status message with new pose
    setCameraPoseUsingROSCoordinates(cam_pose_eigen, 0);
    setCameraPoseUsingROSCoordinates(cam_pose_eigen, 1);

    // Update timestamp of state message (needed to force FlightGoggles to rerender scene)
    state.utime = flightGoggles.getTimestamp();
    // request render
    flightGoggles.requestRender(state);
    
}

///////////////////////
// Example Client Node
///////////////////////

int main(int argc, char **argv) {
    ros::init(argc, argv, "pose_listener");

    ros::NodeHandle node;
    
    // Create client
    SyntheticImagesPublisher client;

    // Load params
    client.populateRenderSettings();

    callback poseCallback = boost::bind(&SyntheticImagesPublisher::poseSubscriber, &client, _1);
    ros::Subscriber sub = node.subscribe("odom", 1, poseCallback);

    // Spin
    ros::spin();

    return 0;
}
