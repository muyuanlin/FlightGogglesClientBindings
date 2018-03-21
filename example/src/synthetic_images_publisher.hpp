#ifndef SYNTHETICIMAGEPUBLISHER_H
#define SYNTHETICIMAGEPUBLISHER_H
/**
 * @file   GeneralClient.hpp
 * @author Winter Guerra
 * @brief  Basic client interface for FlightGoggles.
 */

#include <flight_goggles_client.hpp>
// #include <jsonMessageSpec.hpp>

#include <iostream>
#include <string>
#include <vector>
#include <thread>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"

#include <boost/function.hpp>
#include <boost/bind.hpp>


// For converting ROS/LCM coordinates to Unity coordinates
#include "transforms.hpp"

class SyntheticImagesPublisher {
public:
    // FlightGoggles interface object
	FlightGogglesClient flightGoggles;

	// Base status object (which holds camera settings, env settings, etc)
	unity_outgoing::StateMessage_t state;

	// // constructor
	// SyntheticImagesPublisher(){}

	// Populate starting settings into state
	void populateRenderSettings();

	void setCameraPoseUsingROSCoordinates(Eigen::Affine3d ros_pose, int cam_index);

	void poseSubscriber(const nav_msgs::Odometry::ConstPtr& msg);
private:
    void poseMsgToEigen(const geometry_msgs::Pose &m, Eigen::Affine3d &e)
    {
     	e = Eigen::Translation3d(m.position.x,
                              m.position.y,
                              m.position.z) *
        Eigen::Quaterniond(m.orientation.w,
                          m.orientation.x,
                          m.orientation.y,
                          m.orientation.z);
   }

};

#endif
