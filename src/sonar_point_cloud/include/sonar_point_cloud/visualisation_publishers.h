#ifndef VISUALISATION_PUBLISHERS_H
#define VISUALISATION_PUBLISHERS_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <gtsam/geometry/Pose2.h>
#include "sonar_point_cloud/scan_matcher.h"


// Function declaration
void robot_trajectory_publisher(ros::Publisher &marker_pub, const std::vector<float>& position, visualization_msgs::Marker &points, visualization_msgs::Marker &line_strip, const std::string &ns, bool initialized);

// To visualise optimised poses
void optimised_robot_trajectory(ros::Publisher &marker_pub, const std::vector<std::vector<float>>& optimised_poses, 
                              visualization_msgs::Marker &points, visualization_msgs::Marker &line_strip, 
                              const std::string &ns, bool initialized);

// Function to publish all optimised poses with their covariances
void optim_publishCovariances(ros::Publisher& publisher, const std::vector<std::vector<float>>& poses,
                                const std::vector<gtsam::Matrix>& covariances);

// Function to publish all scan-matching estimated poses with their covariances
void sm_publishCovariances(ros::Publisher& publisher, const std::vector<std::vector<float>>& poses,
                                   const std::vector<Eigen::Matrix3d>& covariances);

// Plot the final MAP
void plotMap(
    const std::vector<PointCloudVariant>& scans,
    const std::vector<std::vector<float>>& gt_transformations,
    const std::vector<std::vector<float>>& estimated_transformations,
    const std::string& method_txt
);

#endif // VISUALISATION_PUBLISHERS_H