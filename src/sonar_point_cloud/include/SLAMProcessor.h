#ifndef SLAM_PROCESSOR_H
#define SLAM_PROCESSOR_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <variant>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <utility> 
#include <iostream>
#include <filesystem>
#include <mutex>
#include <cmath>
#include <algorithm>
#include <omp.h>
#include "matplotlibcpp.h"
#include "sonar_point_cloud/point_cloud_extractor.h"
#include "sonar_point_cloud/scan_matcher.h"
#include "sonar_point_cloud/visualisation_publishers.h"
#include "sonar_point_cloud/helper_functions.h"

// GMM
#include <gmm_registration/eigen_fix.h>
#include <gmm_registration/front_end/GaussianMixturesModel.h>
#include <gmm_registration/front_end/GmmFrontEnd.hpp>
#include <gmm_registration/method/PointsToDistribution3D.h>
#include <gmm_registration/method/ScanMatchingMethods.h>
#include <gmm_registration/solver/CholeskyLineSearchNewtonMethod.h>
#include <gmm_registration/solver/Solver.h>



// pIC_registration
#include "pIC_registration.h"
#include "pIC_registration_terminate.h"
#include "rt_nonfinite.h"
#include "coder_array.h"

// GraphBuilder
#include "GraphBuilder.h"


// Visualising the Graph
#include "graph_slam_visualizer/GraphSE2.h"

#include "cirs_sam_navigator_public/slam/ScanMatchingSlam.h"
#include "cirs_sam_navigator_public/PoseWithCovarianceArray.h"

namespace plt = matplotlibcpp;

class SLAMProcessor {
public:
    SLAMProcessor();

    void callback(const sensor_msgs::ImageConstPtr& msg);
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void pose_update_Callback(const cirs_sam_navigator_public::PoseWithCovarianceArray::ConstPtr& msg);
    void displayImages();
    bool getRobotPoseAtTime(tf::TransformListener& listener, const ros::Time& timestamp, geometry_msgs::PoseStamped& pose, float& yaw_angle);
    std::vector<std::vector<float>> graph_optimised_poses_{{0.0f,0.0f,0.0f}}; // result of graph slam
    std::vector<gtsam::Matrix> covariances{gtsam::Matrix::Zero(3, 3)}; // graph_optimised_poses_ covariances

private:
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    ros::Subscriber odom_sub_;
    std::vector<float> x_;
    std::vector<float> y_;
    std::tuple<float, float, float> estimated_transformation;
    std::mutex data_mutex_;
    std::mutex mutex1;
    std::mutex mutex2;
    cv::Mat cartesian_image_;
    bool full_scan_taken_= false;
    std::vector<double> old_column_sums;
    std::vector<double> column_sums;
    std::vector<float> key_pose;

    float radius_;
    float horizontal_FOV_;
    int intensity_threshold_;
    std::vector<float> previous_pose_{0.0f, 0.0f, 0.0f};
    float odom_distance_threshold_;
    float odom_angle_threshold_;
    float min_sonar_range_;
    double sonar_noise_;
    float sonar_roll_;
    float sonar_x_position_; // x axis position of the sonar in the girona500/base_link frame
    float sonar_y_position_; // y axis position of the sonar in the girona500/base_link frame
    float confidence_level_; // Test for scan-matching measurement compatibility.
    float loop_closure_confi_level_; // Confidence level indicating loop closure between two poses
    bool take_new_scan_ = true;
    std::string algorithm_;  // scan-mathing algorithms= icp, gicp, gmm, pIC
    int num_points_per_beam_;
    Eigen::Matrix3d registration_estimated_cov_= Eigen::Matrix3d::Zero(3, 3); // the estmated registration uncertainty

    // To store the scans in the format PCL library, pIC and GMM expects
    using PointCloudVariant = std::variant
    <
        std::vector<Eigen::Vector2d>,             // for gmm
        pcl::PointCloud<pcl::PointXYZ>,           // for ICP and GICP
        coder::array<double, 2U>                  // for pIC
    >;

    PointCloudVariant cloud_;
    std::vector<PointCloudVariant> intermediate_scans_;
    std::vector<std::vector<PointCloudVariant>> scans_;
    std::vector<PointCloudVariant> full_scans_; // when the individual scans are compounded to form full 360 degree scan
    std::vector<std::vector<std::vector<float>>> intermediate_poses_; // poses at which scans are gathered to form full scan poses={ scan1{ {x1,y1,yaw1}, {x2,y2,yaw2}, {x3,y3,yaw3} }, scan2{ {x1,y1,yaw1}, {x2,y2,yaw2}, {x3,y3,yaw3} }, scan4....}
    std::vector<std::vector<float>> GT_intermediate_poses_;
    std::vector<std::vector<float>> GT_poses_{{0.0f,0.0f,0.0f}}; // Ground Truth poses
    std::vector<std::vector<float>> scan_mathing_estimated_poses_{{0.0f,0.0f,0.0f}}; // Poses estimated by scan matching algorithms without initial estimates
    

    // Covariances
    std::vector<Eigen::Matrix3d> scan_mathing_estimated_poses_covs_= {Eigen::Matrix3d::Zero(3, 3)}; // cov of  the poses estimated by scan-matching based dead reckoning

    // For visualisation in RVIZ using the visualisation_publishers.cpp
    ros::Publisher sm_result_pub_; // Scan matching result publisher
    ros::Publisher gt_marker_pub_; // ground truth marker publisher
    ros::Publisher sm_marker_pub_; // scan matching algorithms estimated pose marker publisher
    ros::Publisher optimised_marker_pub_; // optimised poses marker publisher
    ros::Publisher sm_cov_pub_; // covariances of scan-matching poses publisher
    ros::Publisher optimised_cov_pub_; // covariances of the optimised poses publisher
    visualization_msgs::Marker gt_points_, gt_line_strip_, sm_points_, sm_line_strip_, optim_points_, optim_line_strip_;
    ros::Publisher graph_pub_;

    // GraphBuilder
    GraphBuilder graph_builder_; // initialise the graph... it is assumed it starts at the origin and no uncertainty in the starting pose
   

    tf::TransformListener listener_; // tf listner to get robot pose at some specific time in the past
    std::vector<ros::Time> timestamps_; // to store time stamps of the scans
    bool isCompatible;

    cv::Mat old_raw_image;
    int num_beams_updated_;
    int num_scans_taken = 0; // number of scans taken

};

#endif // SLAM_PROCESSOR_H

