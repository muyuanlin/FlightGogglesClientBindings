/**
 * @file  trajectory_generation.cpp
 * @author Muyuan Lin
 * @brief  Publish simulated pose and velocity of vechicles.
 *
 **/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "trajectory_publisher");

    ros::NodeHandle n;
    ros::Publisher trajectory_pub = n.advertise<nav_msgs::Odometry>("trajectory", 50);
    tf::TransformBroadcaster trajectory_broadcaster;

    double x = 0.0;
    double y = -1.0;
    double z = 0.2;
    double th = 0.0;

    double vx = 0.3;
    double vy = -0.3;
    double vz = 0.0;
    double vth = 0.2;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate rate(10);
    while(n.ok()){

        ros::spinOnce();               // check for incoming messages
        current_time = ros::Time::now();

        //compute odometry in a typical way given the velocities of the robot
        double dt = (current_time - last_time).toSec();
        double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        double delta_th = vth * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion orientation_quat = tf::createQuaternionMsgFromYaw(th);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "map";
        odom_trans.child_frame_id = "velocity";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = z;
        odom_trans.transform.rotation = orientation_quat;

        //send the transform
        trajectory_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the trajectory message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "map";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = z;
        odom.pose.pose.orientation = orientation_quat;

        //set the velocity
        odom.child_frame_id = "velocity";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;

        //publish the message
        trajectory_pub.publish(odom);

        last_time = current_time;
        rate.sleep();
    }
}
