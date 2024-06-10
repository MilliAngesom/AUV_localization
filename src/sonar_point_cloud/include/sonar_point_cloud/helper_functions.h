#ifndef HELPER_FUNCTIONS_H
#define HELPER_FUNCTIONS_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "GraphBuilder/GraphBuilder.h"
#include "graph_slam_visualizer/GraphSE2.h"
#include "graph_slam_visualizer/EdgeSE2.h"
#include "graph_slam_visualizer/VertexSE2.h"
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <variant>
#include <string>
#include "coder_array.h"



using PointCloudVariant = std::variant<
    std::vector<Eigen::Vector2d>,
    pcl::PointCloud<pcl::PointXYZ>,
    coder::array<double, 2U>
>;

std::vector<int> detect_loop_closure(const std::vector<std::vector<float>>& poses, const std::vector<float>& new_pose,
                                     const gtsam::Matrix& cov, const float confidence_level);

void publishGraph(ros::Publisher &pub, const gtsam::NonlinearFactorGraph &graph, const gtsam::Values &initialEstimates);

bool isMeasurementCompatible(const std::vector<float>& pose1, const std::vector<float>& pose2,
                             const gtsam::Matrix& covariance1, const gtsam::Matrix& covariance2,
                             const std::tuple<float, float, float>& measurement, float confidence_level);

float normalize_angle(float angle);

// to retrive the robot pose at some specific time in the past
bool getRobotPoseAtTime(tf::TransformListener& listener, const ros::Time& timestamp, geometry_msgs::PoseStamped& pose, float& yaw_angle);

// Transform and align the individual scan segments to the central reference frame to construct a comprehensive 360-degree panoramic view.
PointCloudVariant compoundScans( const std::vector<PointCloudVariant>& scans, const std::vector<std::vector<float>>& poses, const int target_pose_index);

// Function to Update Poses and Propagate Uncertainties
void updatePoseAndCovariances(std::vector<std::vector<float>>& Poses, std::vector<Eigen::Matrix3d>& Covariances, 
                const std::tuple<float, float, float>& uk, const Eigen::Matrix3d& new_cov);

// Function to publish scan matching results
void publish_sm_result(ros::Publisher &pub, const std::vector<float> &pose_vector, const Eigen::Matrix3d &covariance);

// Function to publish the current pose of the robot according to the graph slam pose estimates
void publishOdometry(ros::Publisher &pub ,const std::vector<float>& pose);

#endif // HELPER_FUNCTIONS_H