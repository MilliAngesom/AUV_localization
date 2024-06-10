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

// For ICP and GICP
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

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


namespace plt = matplotlibcpp;

class SLAMProcessor {
public:
    SLAMProcessor() :
        nh_("~") {

        // Retrieve parameters from the parameter server and provide default values
        nh_.param<float>("radius", radius_, 20.0);
        nh_.param<float>("horizontal_FOV", horizontal_FOV_, 360.0);
        nh_.param<int>("intensity_threshold", intensity_threshold_, 50);
        nh_.param<std::string>("algorithm", algorithm_, "pic");
        nh_.param<float>("odom_distance_threshold", odom_distance_threshold_, 1.0);
        nh_.param<float>("odom_angle_threshold", odom_angle_threshold_, 0.3);
        nh_.param<float>("min_sonar_range", min_sonar_range_, 1.5);
        nh_.param<int>("num_points_per_beam", num_points_per_beam_, 1);
        nh_.param<double>("sonar_noise", sonar_noise_, 0.1);
        nh_.param<float>("confidence_level", confidence_level_, 0.68);
        nh_.param<float>("loop_closure_confi_level", loop_closure_confi_level_, 0.68);
        nh_.param<float>("sonar_roll_orientation", sonar_roll_, 1.45);
        nh_.param<float>("sonar_x_position", sonar_x_position_, 0.8);
        nh_.param<float>("sonar_y_position", sonar_y_position_, 0.0);

        // Initialize container for storing intermediate poses during SLAM processing
        intermediate_poses_.push_back(std::vector<std::vector<float>>()); // required for proper initialisation
        
        
        
        // Advertise topics for visualization in ROS
        gt_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("gt/visualization_marker", 10); // will be used in "visualisation_publishers.cpp"
        sm_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("sm/visualization_marker", 10); // will be used in "visualisation_publishers.cpp"
        optimised_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("optim/visualization_marker", 10); // will be used in "visualisation_publishers.cpp"
        sm_cov_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("sm/pose_with_covariance", 10);
        optimised_cov_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("optim/pose_with_covariance", 10);
        graph_pub_ = nh_.advertise<graph_slam_visualizer::GraphSE2>("graph_visualisation", 10); // to publish the visualisation of the whole pose graph

        // Subscribe to image and odometry topics
        image_sub_ = nh_.subscribe("/girona500/msis/image", 1, &SLAMProcessor::sonar_image_callback, this);
        odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/girona500/dynamics/odometry", 1, &SLAMProcessor::gt_odom_callback, this);
    }

    void sonar_image_callback(const sensor_msgs::ImageConstPtr& msg) {

        if (take_new_scan_){
            // Declaration of matrices to store images
            cv::Mat raw_image;
            cv::Mat result;

            try {
                // Convert the ROS image message to a cv::Mat using cv_bridge
                raw_image = cv_bridge::toCvShare(msg, "mono8")->image;
            } catch (cv_bridge::Exception& e) {
                // Log an error message if conversion fails
                ROS_ERROR("CvBridge Error: %s", e.what());
                return; // Exit the function on failure
            }

            // Check if this is the first image received
            if (old_raw_image.empty()){
                // Save the first image for comparison with future images
                old_raw_image = raw_image.clone();
                num_beams_updated_=0;  // Initialize the beam update counter
                return;
            }
            else 
            {   
                // Compute the absolute difference between the current and previous images
                cv::absdiff(old_raw_image, raw_image, result);
                // Update the old image to the current one for future comparisons
                old_raw_image = raw_image.clone();
                

                // Iterate through each column of the result image
                for (int col = 0; col < result.cols; ++col) {
                    // Check the sum of pixel values in the column to determine significant changes
                    if (cv::sum(result.col(col))[0]<100){
                        // Set the column in the original image to zero if change is insignificant
                        raw_image.col(col).setTo(cv::Scalar(0));
                    }
                    else {
                        // Increment the count of updated beams if the change is significant
                        num_beams_updated_++;
                    }
                } 
            }

            // Check if a complete 360-degree scan has been achieved
            if (num_beams_updated_>=raw_image.cols){
                full_scan_taken_ = true;  // Indicate a full scan has been completed
                std::cout << "  FULL SCAN TAKEN !!!" << std::endl;
            }
            
            // Clear data structures based on the type stored in the variant 'cloud_'
            if (std::holds_alternative<std::vector<Eigen::Vector2d>>(cloud_)) {
                // Clearing specific operations for vector
                std::get<std::vector<Eigen::Vector2d>>(cloud_).clear();
            } else if (std::holds_alternative<pcl::PointCloud<pcl::PointXYZ>>(cloud_)) {
                // Clearing specific operations for point cloud
                std::get<pcl::PointCloud<pcl::PointXYZ>>(cloud_).clear();
            } else if (std::holds_alternative<coder::array<double, 2U>>(cloud_)) {
                // Clearing specific operations for coder array
                std::get<coder::array<double, 2U>>(cloud_) = coder::array<double, 2U>(); 
            }
            

            {std::lock_guard<std::mutex> lock(data_mutex_);
            x_.clear();  // Clear the x-coordinates vector
            y_.clear();  // Clear the y-coordinates vector
            
            // Convert polar sonar data to Cartesian coordinates and populate 'cloud_'
            polarToCartesian(raw_image, radius_, horizontal_FOV_, static_cast<uint8_t>(intensity_threshold_), min_sonar_range_, x_, y_, 
                             cloud_, num_points_per_beam_, algorithm_, sonar_roll_,sonar_x_position_,sonar_y_position_); 

            
            geometry_msgs::PoseStamped robot_pose_sourcePC;  // PoseStamped to store the robot's pose
            float yaw_angle; // Variable to store the yaw angle
            // Retrieve the robot's pose at the specific timestamp where the sonar scan was taken
            bool pose_found = getRobotPoseAtTime(listener_, msg->header.stamp, robot_pose_sourcePC, yaw_angle);

            // Check if a valid pose was found for the current scan
            if (pose_found)
            // Ensure there is a space in the vectors for a new scan and associated pose
            {   if (num_scans_taken >= scans_.size()) {
                    scans_.push_back(std::vector<PointCloudVariant>()); // Expand the scan storage
                    intermediate_poses_.push_back(std::vector<std::vector<float>>());  // Expand the pose storage
                }
                
                // Store the current scan and its corresponding pose
                scans_[num_scans_taken].push_back(cloud_); // save the intermediate scan
                intermediate_poses_[num_scans_taken].push_back({robot_pose_sourcePC.pose.position.x, robot_pose_sourcePC.pose.position.y, yaw_angle}); // Save the pose data
            }
            else{
                // Output a warning if the pose for the current scan could not be retrieved
                std::cerr << "Warning: Pose at which the scan was taken couldn't be retrieved, returning..." << std::endl;
                return; // Exit the function if no pose could be found
            }
            }
            
            // Check if a full 360-degree scan has been completed
            if (not full_scan_taken_)
            {
                return; // Return if the full scan is not yet complete
            }
            else{
                // Reset scan-related flags and counters if a full scan has been taken
                take_new_scan_ = false;
                full_scan_taken_ = false;
                old_raw_image.release();
                num_scans_taken = num_scans_taken + 1;
                std::cout << "take_new_scan_: " << take_new_scan_ << std::endl; // Output the state of taking a new scan
            }
            
            
            // Actions to perform after the first full scan is completed
            if (num_scans_taken == 1){       
            // Calculate the index for the middle pose to establish a reference     
            auto indx =ceil(intermediate_poses_[num_scans_taken-1].size()/2); // Index of the central pose

             // Save the central pose 
            GT_poses_.push_back(intermediate_poses_[0][indx]); // Ground Truth poses
            scan_mathing_estimated_poses_.push_back(intermediate_poses_[0][indx]); // Poses estimated by scan matching algorithms
            graph_optimised_poses_.push_back(intermediate_poses_[0][indx]); // Result of graph slam

            // Initialize the visualization publishers to output the trajectories to RVIZ
            robot_trajectory_publisher(gt_marker_pub_, GT_poses_.back(), gt_points_, gt_line_strip_, "ground_truth", false); // Publish the ground-truth trajectory
            robot_trajectory_publisher(sm_marker_pub_, scan_mathing_estimated_poses_.back(), sm_points_, sm_line_strip_, "sm", false); // Publish the trajectory estimated by scan-matching algorithm fused with dead reckoning
            optimised_robot_trajectory(optimised_marker_pub_, graph_optimised_poses_, optim_points_, optim_line_strip_, "optimised", false);// Publish the optimised trajectory
            }
            
            else // if more than one scans available
            {   auto indx_tgt =ceil(intermediate_poses_[num_scans_taken-2].size()/2); // pose at which target scan is compounded
                auto indx_src =ceil(intermediate_poses_[num_scans_taken-1].size()/2); // pose at which source scan is compounded

                // Compute an initial guess for the transformation between the target and source poses in the body frame
                std::vector<float> initial_guess{0.0f,0.0f,0.0f};
                initial_guess = {-intermediate_poses_[num_scans_taken-2][indx_tgt][0]*cos(intermediate_poses_[num_scans_taken-2][indx_tgt][2]) - intermediate_poses_[num_scans_taken-2][indx_tgt][1]*sin(intermediate_poses_[num_scans_taken-2][indx_tgt][2]) + intermediate_poses_[num_scans_taken-1][indx_src][0]*cos(intermediate_poses_[num_scans_taken-2][indx_tgt][2])+intermediate_poses_[num_scans_taken-1][indx_src][1]*sin(intermediate_poses_[num_scans_taken-2][indx_tgt][2]),
                                  intermediate_poses_[num_scans_taken-2][indx_tgt][0]*sin(intermediate_poses_[num_scans_taken-2][indx_tgt][2]) - intermediate_poses_[num_scans_taken-2][indx_tgt][1]*cos(intermediate_poses_[num_scans_taken-2][indx_tgt][2]) - intermediate_poses_[num_scans_taken-1][indx_src][0]*sin(intermediate_poses_[num_scans_taken-2][indx_tgt][2])+intermediate_poses_[num_scans_taken-1][indx_src][1]*cos(intermediate_poses_[num_scans_taken-2][indx_tgt][2]),
                                  normalize_angle(intermediate_poses_[num_scans_taken-1][indx_src][2] - intermediate_poses_[num_scans_taken-2][indx_tgt][2])
                                } ;                                                     

               // Compounding scans at given indices to generate a comprehensive 360-degree panoramic view. 
                PointCloudVariant source_cloud = compoundScans( scans_[num_scans_taken-1], intermediate_poses_[num_scans_taken-1], indx_src);
                PointCloudVariant target_cloud = compoundScans( scans_[num_scans_taken-2], intermediate_poses_[num_scans_taken-2], indx_tgt);


                // Register the source and target scans using the initial transformation guess
                std::vector<double> initial_guess_cov = {0.5, 0.5, 0.01};
                estimated_transformation = register_scans(source_cloud, target_cloud, initial_guess, initial_guess_cov,
                                                            sonar_noise_, registration_estimated_cov_, algorithm_);

                // Extract the estimated transformation components (x, y, yaw) from the registration result
                float x_sm, y_sm, yaw_sm;
                std::tie(x_sm, y_sm, yaw_sm) = estimated_transformation;
                
                std::cout << "number of scans: " << scans_.size() << std::endl;  

                //? Check measurement compatibility
                // J1 - partial derivation of motion model with respect to previous pose Xt-1 
                Eigen::Matrix3d J1;
                J1 << 1, 0, -x_sm*sin(graph_optimised_poses_.back()[2]) - y_sm*cos(graph_optimised_poses_.back()[2]),
                      0, 1,  x_sm*cos(graph_optimised_poses_.back()[2]) - y_sm*sin(graph_optimised_poses_.back()[2]),
                      0, 0,  1;
                
                // J2 - partial derivation of motion model with respect to scan-matching measurements x_sm, y_sm and yaw_sm
                Eigen::Matrix3d J2;
                J2 << cos(graph_optimised_poses_.back()[2]), -sin(graph_optimised_poses_.back()[2]), 0,
                      sin(graph_optimised_poses_.back()[2]),  cos(graph_optimised_poses_.back()[2]), 0,
                        0,                                     0,                                    1;

                std::vector<float> predicted_pose = intermediate_poses_[num_scans_taken-1][indx_src];
                
                
                // convert gtsam::Matrix to Eigen::Matrix3f to avoid future problems
                Eigen::Matrix3d target_pose_cov = covariances.back().cast<double>();
                auto covar = J1 * target_pose_cov * J1.transpose() + 
                                                    J2 * registration_estimated_cov_* J2.transpose();
                
                // Check if registration result is compatible
                isCompatible = isMeasurementCompatible(graph_optimised_poses_.back(), predicted_pose, 
                                                    covariances.back(), covar, estimated_transformation, confidence_level_);  
                
                std::cout << "Odometry compatible: " << isCompatible << std::endl;
                // If the measurement is not compatible, use the initial estimation (dead reckoning)
                if ( not isCompatible){
                    x_sm = initial_guess[0];
                    y_sm = initial_guess[1];
                    yaw_sm=initial_guess[2];
                }
                
                // Store the new pose estimated by the scan-matching algorithm or dead reckoning estimation
                float theta = scan_mathing_estimated_poses_.back()[2];
                float delta_x = cos(theta) * x_sm - sin(theta) * y_sm;
                float delta_y = sin(theta) * x_sm + cos(theta) * y_sm;
                std::vector<float> new_estimated_pose = {scan_mathing_estimated_poses_.back()[0] + delta_x, 
                                                         scan_mathing_estimated_poses_.back()[1] + delta_y,
                                         normalize_angle(scan_mathing_estimated_poses_.back()[2] + yaw_sm)
                                            };
                scan_mathing_estimated_poses_.push_back(new_estimated_pose);




                // Update the covariance for the new pose based on scan-matching or dead reckoning and publish to RVIZ
                // J1 - partial derivation of motion model with respect to previous pose Xt-1 
                J1 << 1, 0, -x_sm*sin(scan_mathing_estimated_poses_[scan_mathing_estimated_poses_.size()-2][2]) - y_sm*cos(scan_mathing_estimated_poses_[scan_mathing_estimated_poses_.size()-2][2]),
                      0, 1,  x_sm*cos(scan_mathing_estimated_poses_[scan_mathing_estimated_poses_.size()-2][2]) - y_sm*sin(scan_mathing_estimated_poses_[scan_mathing_estimated_poses_.size()-2][2]),
                      0, 0,  1;
                
                // J2 - partial derivation of motion model with respect to scan-matching measurements x_sm, y_sm and yaw_sm
                J2 << cos(scan_mathing_estimated_poses_[scan_mathing_estimated_poses_.size()-2][2]), -sin(scan_mathing_estimated_poses_[scan_mathing_estimated_poses_.size()-2][2]), 0,
                      sin(scan_mathing_estimated_poses_[scan_mathing_estimated_poses_.size()-2][2]),  cos(scan_mathing_estimated_poses_[scan_mathing_estimated_poses_.size()-2][2]), 0,
                      0, 0,  1;

                Eigen::Matrix3d cov;
                cov = J1 * scan_mathing_estimated_poses_covs_.back() * J1.transpose() + J2 * registration_estimated_cov_* J2.transpose();
                
                scan_mathing_estimated_poses_covs_.push_back(cov);

                sm_publishCovariances(sm_cov_pub_, scan_mathing_estimated_poses_,scan_mathing_estimated_poses_covs_);


                // Publish the new pose in RVIZ using robot trajectory publisher
                robot_trajectory_publisher(sm_marker_pub_, new_estimated_pose, sm_points_, sm_line_strip_, "sm", true);


                // ADD THE NEW POSE TO THE GRAPH AND OPTIMISE THE GRAPH 
                theta = graph_optimised_poses_.back()[2];
                delta_x = cos(theta) * x_sm - sin(theta) * y_sm;
                delta_y = sin(theta) * x_sm + cos(theta) * y_sm;
                new_estimated_pose = {graph_optimised_poses_.back()[0] + delta_x, 
                                      graph_optimised_poses_.back()[1] + delta_y,
                      normalize_angle(graph_optimised_poses_.back()[2] + yaw_sm)
                                            };
                
               
                
                auto sm_noiseModel_ = gtsam::noiseModel::Gaussian::Covariance(registration_estimated_cov_);

                // Adding poses and odometry constraints to the graph
                graph_builder_.addPose(gtsam::Pose2(new_estimated_pose[0], new_estimated_pose[1], new_estimated_pose[2]), gtsam::Pose2(x_sm, y_sm, normalize_angle(yaw_sm)), sm_noiseModel_);
                auto Cov_current_pose = graph_builder_.getMarginalCovariances(graph_builder_.currentIndex); 

                // Check for potential loop closure based on the confidence level and current estimated pose
                std::vector<int> loop_closure_candidates= detect_loop_closure(graph_optimised_poses_, new_estimated_pose ,Cov_current_pose.at(0), loop_closure_confi_level_);
                bool true_loop_closure_found= false;  // Flag to indicate if a true loop closure has been confirmed

                // If potential loop closure candidates exist
                if (loop_closure_candidates.size()>0){
                    // Determine the current source scan's index for loop closure checking
                    indx_src =ceil(intermediate_poses_[num_scans_taken-1].size()/2); // pose at which source scan is compounded
                    
                    // Iterate through each loop closure candidate to verify and register scans
                    for(int indx: loop_closure_candidates){
                        std::cout << "loop closure detected with: "<<indx << std::endl;
                        // Calculate the target index from the potential loop closure candidate
                        indx_tgt =ceil(intermediate_poses_[indx].size()/2); // pose at which target scan is compounded
                    
                        // Compute the initial guess for the transformation in the body frame based on the source and target data
                        initial_guess = {-intermediate_poses_[indx][indx_tgt][0]*cos(intermediate_poses_[indx][indx_tgt][2]) - intermediate_poses_[indx][indx_tgt][1]*sin(intermediate_poses_[indx][indx_tgt][2]) + intermediate_poses_[num_scans_taken-1][indx_src][0]*cos(intermediate_poses_[indx][indx_tgt][2])+intermediate_poses_[num_scans_taken-1][indx_src][1]*sin(intermediate_poses_[indx][indx_tgt][2]),
                                          intermediate_poses_[indx][indx_tgt][0]*sin(intermediate_poses_[indx][indx_tgt][2]) - intermediate_poses_[indx][indx_tgt][1]*cos(intermediate_poses_[indx][indx_tgt][2]) - intermediate_poses_[num_scans_taken-1][indx_src][0]*sin(intermediate_poses_[indx][indx_tgt][2])+intermediate_poses_[num_scans_taken-1][indx_src][1]*cos(intermediate_poses_[indx][indx_tgt][2]),
                                          normalize_angle(intermediate_poses_[num_scans_taken-1][indx_src][2] - intermediate_poses_[indx][indx_tgt][2])
                                        } ;                                                     

                        // scan forming to construct a comprehensive 360-degree panoramic view. 
                        target_cloud = compoundScans( scans_[indx], intermediate_poses_[indx], indx_tgt);

                        // Register the scans to compute the estimated transformation
                        estimated_transformation = register_scans(source_cloud, target_cloud, initial_guess, initial_guess_cov,
                                                                    sonar_noise_, registration_estimated_cov_, algorithm_);

                        std::tie(x_sm, y_sm, yaw_sm) = estimated_transformation; // Extract transformation result

                       
                        // Check measurement compatibility
                        isCompatible = isMeasurementCompatible(graph_optimised_poses_.at(indx), new_estimated_pose, 
                                                            covariances.at(indx), Cov_current_pose.at(0), estimated_transformation, confidence_level_);
                        std::cout << "loop closure isCompatible?: " << isCompatible << std::endl;
                        if (isCompatible){

                            std::cout << "Measurement is compatible!!!" << std::endl;
                            auto sm_noiseModel_ = gtsam::noiseModel::Gaussian::Covariance(registration_estimated_cov_);
                            // If compatible, add a loop closure constraint to the SLAM graph
                            graph_builder_.addLoopClosure(indx+1, scans_.size(),  gtsam::Pose2(x_sm, y_sm, normalize_angle(yaw_sm)), sm_noiseModel_);
                            // Set the flag as a valid loop closure is found
                            true_loop_closure_found = true;
                        }
                    }
                }

                // After verifying and registering all candidates
                if (true_loop_closure_found)
                {
                    // Optimize the graph once a true loop closure is added
                    graph_builder_.optimizeGraph();

                    // Update and retrieve the optimized poses
                    graph_builder_.update_poses(graph_optimised_poses_);

                    // Retrieve the new covariances after optimization
                    covariances = graph_builder_.getMarginalCovariances();
                }
                else { 
                    // If no true loop closure is found, continue appending the estimated pose and its covariance
                    graph_optimised_poses_.push_back(new_estimated_pose);
                    covariances.push_back(Cov_current_pose.at(0));
                    
                }
                
                
                // Publish the optimised poses to rviz
                optimised_robot_trajectory(optimised_marker_pub_, graph_optimised_poses_, optim_points_, optim_line_strip_, "optimised", true);

                // Publish all poses' covariances to Rviz
                optim_publishCovariances(optimised_cov_pub_, graph_optimised_poses_, covariances);

                // visualise the graph in rviz
                publishGraph(graph_pub_, graph_builder_.graph, graph_builder_.initialEstimates);
                
            }
            
        }
    }


    void gt_odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
        
        // Access message data
        float posX = static_cast<float>(msg->pose.pose.position.x);
        float posY = static_cast<float>(msg->pose.pose.position.y);
        tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
        );
        // Convert the quaternion to a yaw angle, normalize it to ensure it is within the range of -pi to pi
        float posYaw = static_cast<float>(normalize_angle(tf::getYaw(q)));

        // Calculate the Euclidean distance from the last recorded ground truth position to the current odometry position
        float dist = std::sqrt(std::pow(GT_poses_.back()[0] - posX, 2) + std::pow(GT_poses_.back()[1] - posY, 2));

        // Check if the robot has moved beyond the distance threshold or the angle threshold and if a new scan is not already flagged
        if ((dist > odom_distance_threshold_ || std::abs(GT_poses_.back()[2] - posYaw) > odom_angle_threshold_) && (not take_new_scan_)) {
            // Allow taking new scan 
            take_new_scan_ = true;
            std::cout << "take_new_scan_: " << take_new_scan_ << std::endl;
            
        }

        if ( take_new_scan_) {
            temporary_gt_pose_storage_.push_back({posX, posY, posYaw});        
        }

        else if (not take_new_scan_ && not temporary_gt_pose_storage_.empty()){
            auto indx = ceil(temporary_gt_pose_storage_.size()/2);
            GT_poses_.push_back(temporary_gt_pose_storage_[indx]);
            temporary_gt_pose_storage_.clear();
            robot_trajectory_publisher(gt_marker_pub_, GT_poses_.back(), gt_points_, gt_line_strip_, "ground_truth", true); // ground-truth trajectory
        }
        
    }

    // to retrive the robot pose at some specific time in the past
    bool getRobotPoseAtTime(tf::TransformListener& listener, const ros::Time& timestamp, geometry_msgs::PoseStamped& pose, float& yaw_angle) {
        tf::StampedTransform transform;
        try {
            // Specify the time 
            listener.waitForTransform("world_ned", "girona500/base_link", timestamp, ros::Duration(1.0));
            listener.lookupTransform("world_ned", "girona500/base_link", timestamp, transform);

            // Fill the PoseStamped message with the transform data
            pose.header.stamp = transform.stamp_;
            pose.header.frame_id = "world_ned";
            pose.pose.position.x = transform.getOrigin().x();
            pose.pose.position.y = transform.getOrigin().y();
            pose.pose.position.z = transform.getOrigin().z();
            pose.pose.orientation.x = transform.getRotation().x();
            pose.pose.orientation.y = transform.getRotation().y();
            pose.pose.orientation.z = transform.getRotation().z();
            pose.pose.orientation.w = transform.getRotation().w();

            // Get the yaw angle from the quaternion
            yaw_angle = normalize_angle(tf::getYaw(transform.getRotation()));

            return true;
        } catch (tf::TransformException& ex) {
            // ROS_ERROR("%s", ex.what());
            return false;
        }
    }

             


    void run() 
    {
        while (ros::ok()) 
        {
            ros::spinOnce();
        }
    }

    


private:
    ros::NodeHandle nh_; // ROS NodeHandler

    // Subscribers for image and odometry topics
    ros::Subscriber image_sub_;
    ros::Subscriber odom_sub_;

    // Vectors to store x and y coordinates
    std::vector<float> x_;
    std::vector<float> y_;

    // Tuple to store the estimated transformation result of scan matching
    std::tuple<float, float, float> estimated_transformation;

    // Mutexes for thread-safe data handling
    std::mutex data_mutex_;
    std::mutex mutex1;
    std::mutex mutex2;


    // Boolean to track whether a full scan has been successfully taken
    bool full_scan_taken_= false;

    // Sonar and scan parameters
    float radius_;                                          // Sonar range radius
    float horizontal_FOV_;                                  // Horizontal field of view for the sonar
    int intensity_threshold_;                               // Threshold for intensity filtering in images
    float odom_distance_threshold_;                         // Distance threshold to take new scan
    float odom_angle_threshold_;                            // Angle threshold to take new scan
    float min_sonar_range_;                                 // Minimum effective range of the sonar
    double sonar_noise_;                                    // Noise level in sonar measurements
    float sonar_roll_;                                      // Roll orientation of the sonar in the base_link
    float sonar_x_position_;                                // x axis position of the sonar in the girona500/base_link frame
    float sonar_y_position_;                                // y axis position of the sonar in the girona500/base_link frame
    float confidence_level_;                                // Confidence level for scan matching measurement compatibility
    float loop_closure_confi_level_;                        // Confidence level indicating potential for loop closure between poses
    bool take_new_scan_ = true;                             // Flag to control when new scans are initiated
    std::string algorithm_;                                 // Name of the scan-matching algorithm to use (e.g., ICP, GICP, GMM, pIC)
    int num_points_per_beam_;                               // Number of points sampled per sonar beam

    // Estimated covariance of the registration uncertainty
    Eigen::Matrix3d registration_estimated_cov_= Eigen::Matrix3d::Zero(3, 3); // the estmated registration uncertainty initialisation

    // Variant to handle different types of point clouds as per the algorithm requirements
    using PointCloudVariant = std::variant
    <
        std::vector<Eigen::Vector2d>,             // for gmm
        pcl::PointCloud<pcl::PointXYZ>,           // for ICP and GICP
        coder::array<double, 2U>                  // for pIC
    >;

    PointCloudVariant cloud_;
    
    // Storage for scans and poses
    std::vector<std::vector<PointCloudVariant>> scans_;
    std::vector<std::vector<float>> temporary_gt_pose_storage_;
    std::vector<std::vector<std::vector<float>>> intermediate_poses_; // poses at which scans are gathered to form full scan poses={ scan1{ {x1,y1,yaw1}, {x2,y2,yaw2}, {x3,y3,yaw3} }, scan2{ {x1,y1,yaw1}, {x2,y2,yaw2}, {x3,y3,yaw3} }, scan4....}
    std::vector<std::vector<float>> GT_poses_{{0.0f,0.0f,0.0f}};      // Ground Truth poses
    std::vector<std::vector<float>> scan_mathing_estimated_poses_;    // Poses estimated by scan matching algorithms fused with the dead reckoning poses
    std::vector<std::vector<float>> graph_optimised_poses_;           // result of graph slam

    // Covariances
    std::vector<gtsam::Matrix> covariances{gtsam::Matrix::Zero(3, 3)};                                  // Optimized pose covariances
    std::vector<Eigen::Matrix3d> scan_mathing_estimated_poses_covs_= {Eigen::Matrix3d::Zero(3, 3)};     // Covariance of scan-matching estimates

    // ROS Publishers for visualizing different components in RVIZ
    ros::Publisher gt_marker_pub_;         // Publisher for ground truth markers
    ros::Publisher sm_marker_pub_;         // Publisher for scan matching estimated pose markers
    ros::Publisher optimised_marker_pub_;  // Publisher for optimized pose markers
    ros::Publisher sm_cov_pub_;            // Publisher for covariance of scan-matching poses
    ros::Publisher optimised_cov_pub_;     // Publisher for covariance of optimized poses
    visualization_msgs::Marker gt_points_, gt_line_strip_, sm_points_, sm_line_strip_, optim_points_, optim_line_strip_;

    // ROS Publisher for visualizing the entire graph
    ros::Publisher graph_pub_;

    // GraphBuilder instance to manage SLAM graph construction and optimization
    GraphBuilder graph_builder_; 
   
    // tf::TransformListener to access past robot poses
    tf::TransformListener listener_; 
    
    // Boolean to track compatibility of measurements
    bool isCompatible;

    // Matrix to hold the previous raw image for image differencing in Sonar Image processing
    cv::Mat old_raw_image;

    // Counters to track updates and scans
    int num_beams_updated_;
    int num_scans_taken = 0; // Total number of scans taken
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "SLAM_Master_Node");
    SLAMProcessor SlamNode;
    SlamNode.run();
    return 0;
}
