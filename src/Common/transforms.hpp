////////////////////////////////
// TRANSFORMS
////////////////////////////////

// For transforms
#include <Eigen/Geometry>
#include <Eigen/Dense>

#define PRECISION 10
#define DONTALIGNCOLS 1

// define some eigen types and formats
typedef Eigen::Vector3d Vector3;
typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Quaternion<double> Quaternionx;
typedef Eigen::Affine3d Transform3;
typedef Eigen::Matrix4d Matrix4;
typedef Eigen::Matrix3d Matrix3;
Eigen::IOFormat CSV(PRECISION, DONTALIGNCOLS, ",", ",", "", "", "", "");


    // This function converts right hand rule pose to left handed rule coordinates
    // (as used by Unity).
    Transform3 convertGlobalPoseToGlobalUnityCoordinates(
        Transform3 world_T_window, Transform3 unityWorld_T_world) {
        // Switch axis to unity axis.
        Matrix4 unity_pose_T_LCM_pose;
        // x->z, y->x, z->-y
        // clang-format off
        unity_pose_T_LCM_pose <<    0, 1, 0,  0,
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
        Transform3 world_T_body, Transform3 body_T_cam,
        Transform3 unityWorld_T_world) {
        
        Transform3 world_T_cam = world_T_body * body_T_cam;
        Matrix4 cam_T_unitycam;
        // clang-format off
        cam_T_unitycam <<   0, 1, 0, 0,
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