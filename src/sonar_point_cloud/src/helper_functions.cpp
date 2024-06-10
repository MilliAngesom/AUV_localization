#include <cmath>
#include <vector>
#include <tuple>
#include <Eigen/Dense>
#include "GraphBuilder/GraphBuilder.h" 
#include "sonar_point_cloud/helper_functions.h"
#include "matplotlibcpp.h"
#include <mutex>
#include <variant>
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include "coder_array.h"

/**
 * Normalizes an angle to the range [-π, π].
 *
 * This function adjusts an input angle so that it falls within the canonical
 * representation of angles used in many geometrical calculations, where
 * angles wrap around after reaching a full circle (360 degrees or 2π radians).
 * This normalization is particularly useful for reducing calculation errors
 * in functions that compare angles or compute differences between them.
 *
 * @param angle A float representing an angle in radians.
 * @return The normalized angle within the range [-π, π].
 */
float normalize_angle(float angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

/**
 * Checks if a measurement is compatible with the expected transformation derived from poses and their uncertainties.
 * @param pose1 The first pose, the target pose.
 * @param pose2 The second pose, the new pose to evaluate.
 * @param covariance1 The uncertainty (covariance matrix) associated with pose1.
 * @param covariance2 The uncertainty (covariance matrix) associated with pose2.
 * @param measurement A tuple representing the observed transformation (x, y, yaw).
 * @param confidence_level The statistical confidence level for the compatibility check.
 * @returns true if the measurement falls within the expected range given the confidence level, false otherwise.
 */
bool isMeasurementCompatible(const std::vector<float>& pose1, const std::vector<float>& pose2,
                             const gtsam::Matrix& covariance1, const gtsam::Matrix& covariance2,
                             const std::tuple<float, float, float>& measurement, float confidence_level) {
    // Extract pose components
    float x1 = pose1[0], y1 = pose1[1], theta1 = pose1[2];
    float x2 = pose2[0], y2 = pose2[1], theta2 = pose2[2];

    // Calculate expected transformation
    float expected_x = -x1*cos(theta1) - y1*sin(theta1) + x2*cos(theta1) + y2*sin(theta1);
    float expected_y =  x1*sin(theta1) - y1*cos(theta1) - x2*sin(theta1) + y2*cos(theta1);
    float expected_theta = normalize_angle(-theta1 + theta2);


    // Compute the jacobians J_minus and J_Oplus

    Eigen::Matrix3f J_minus;
    Eigen::Matrix3f J_Oplus;
    

    J_minus << -cos(theta1), -sin(theta1), x1*sin(theta1)-y1*cos(theta1)-x2*sin(theta1)+y2*cos(theta1),
                sin(theta1), -cos(theta1), x1*cos(theta1)+y1*sin(theta1)-x2*cos(theta1)-y2*sin(theta1),
                 0,             0,             -1;

   
    J_Oplus << cos(theta1), sin(theta1), 0,
              -sin(theta1), cos(theta1), 0,
               0, 0, 1;

    
    // convert gtsam::Matrix to Eigen::Matrix3f to avoid future problems
    Eigen::Matrix3f cov1 = covariance1.cast<float>(); // cov of the first pose--- target pose
    Eigen::Matrix3f cov2 = covariance2.cast<float>(); // cov of the second pose --source pose

    // propagate the uncertainty
    Eigen::Matrix3f total_uncertainty = J_minus * cov1 * J_minus.transpose() + J_Oplus * cov2 * J_Oplus.transpose();

    
    // Extract measurement components
    float measurement_x = std::get<0>(measurement);
    float measurement_y = std::get<1>(measurement);
    float measurement_theta = normalize_angle(std::get<2>(measurement));

    // Create a vector<float> to store the error
    std::vector<float> error = {0.0, 0.0, 0.0};
    // Compute discrepancies
    error[0] = std::abs(expected_x - measurement_x);
    error[1] = std::abs(expected_y - measurement_y);
    error[2] = std::abs(normalize_angle(expected_theta - measurement_theta));


    // Convert std::vector<float> to Eigen::Vector3f
    Eigen::Vector3f eigenVector_error = Eigen::Map<Eigen::Vector3f>(error.data());

    // compute the mahalanobis distance
    float squared_mahalanobis_dist = eigenVector_error.transpose() * total_uncertainty.inverse() * eigenVector_error;
    

    float critical_value;
    
    // for 3 DOF
    if (confidence_level==0.68f){
    critical_value = 3.5059;} // confidence level of 0.68

    else if (confidence_level==0.90f){
    critical_value = 6.2514;}// confidence level of 0.90

    else if (confidence_level==0.95f){
    critical_value = 7.8147;}// confidence level of 0.95
    
    else if (confidence_level==0.99f){
    critical_value = 11.3449;}// confidence level of 0.99 
    
    else {std::cerr << "Warning: Currently 0.68, 0.90, 0.95 and 0.99 confidence_levels are only supported. Please choose a valid confidence_level. Returning False." << std::endl;
    return false;}

    ROS_INFO_STREAM("............................ Squared mahalanobis distance " << squared_mahalanobis_dist);
    ROS_INFO_STREAM("............................ innovation " << eigenVector_error.norm() << " m");

    return (squared_mahalanobis_dist <=  critical_value);
}


/**
 * Retrieves the robot's pose at a specific time in the past from the transform tree.
 * This function queries the ROS transform system to get the robot's position and orientation at a given time.
 *
 * @param listener A reference to a tf::TransformListener that listens to and caches transformations.
 * @param timestamp The specific time at which the pose should be retrieved.
 * @param pose A reference to a geometry_msgs::PoseStamped to store the retrieved pose.
 * @param yaw_angle A reference to a float to store the yaw angle extracted from the pose's orientation.
 * @return bool True if the pose was successfully retrieved, false if an error occurred (e.g., transform unavailable).
 */
bool getRobotPoseAtTime(tf::TransformListener& listener, const ros::Time& timestamp, geometry_msgs::PoseStamped& pose, float& yaw_angle) {
    tf::StampedTransform transform;
    try {
        // Specify the time you are interested in
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
        ROS_ERROR("%s", ex.what());
        return false;
    }
}




using PointCloudVariant = std::variant<
    std::vector<Eigen::Vector2d>,
    pcl::PointCloud<pcl::PointXYZ>,
    coder::array<double, 2U>
>;

/**
 * Computes a 3x3 homogeneous transformation matrix from a 2D pose.
 * The pose includes a position (x, y) and a rotation (theta) about the z-axis.
 *
 * @param pose A vector of floats representing the pose, where:
 *        pose[0] = x (translation along the x-axis),
 *        pose[1] = y (translation along the y-axis),
 *        pose[2] = theta (rotation about the z-axis in radians).
 * @return A 3x3 transformation matrix of type Eigen::Matrix3f.
 */
Eigen::Matrix3f computeTransformationMatrix(const std::vector<float>& pose) {
    // Compute the cosine and sine of the rotation angle
    float cosTheta = cos(pose[2]);
    float sinTheta = sin(pose[2]);

    // Define the transformation matrix in homogeneous coordinates
    Eigen::Matrix3f transformation;
    transformation << cosTheta, -sinTheta, pose[0],
                      sinTheta,  cosTheta, pose[1],
                           0.0,       0.0, 1.0;
    return transformation;
}

/**
 * Combines multiple point clouds from various poses into a single coordinate frame specified by target_pose_index.
 * The function supports different types of point clouds (GMM, PCL, and pIC).
 *
 * @param scans Vector of point clouds in variant form to handle different point cloud structures.
 * @param poses Vector of vector<float> where each sub-vector represents a pose (x, y, yaw).
 * @param target_pose_index Index of the pose in poses vector that serves as the new coordinate frame's origin.
 * @return PointCloudVariant A variant that can hold different types of point clouds, depending on what was processed.
 * @throws std::out_of_range if the target_pose_index is not within the valid range of poses.
 * @throws std::runtime_error if no valid point cloud data is found after processing.
 */
PointCloudVariant compoundScans( const std::vector<PointCloudVariant>& scans, const std::vector<std::vector<float>>& poses, const int target_pose_index) 
{
    // Check if the target pose index is within the valid range
    if (target_pose_index >= poses.size()) {
        throw std::out_of_range("Target pose index is out of range.");
    }
    // Compute the transformation matrix for the target pose and its inverse
    auto target_pose = computeTransformationMatrix(poses[target_pose_index]);
    auto target_pose_inverse = target_pose.inverse(); 
    // Initialize containers for different types of point cloud data
    std::vector<Eigen::Vector2d> compounded_gmm;    // For Gaussian Mixture Models (GMM)
    pcl::PointCloud<pcl::PointXYZ> compounded_pcl;  // For PCL point clouds
    std::vector<double> x, y;                       // Temporary storage for coordinates for pIC

    // Process each scan depending on its type
    for (size_t i = 0; i < scans.size(); ++i) {
        std::visit([&, i](auto& scan) {
            using T = std::decay_t<decltype(scan)>;
            if constexpr (std::is_same_v<T, std::vector<Eigen::Vector2d>>) {
                // Process scan data --> a vector of 2D points for GMM
                auto pose = computeTransformationMatrix(poses[i]);
                for (auto& point : scan) {
                    Eigen::Vector3f pt(point[0], point[1], 1);
                    Eigen::Vector3f transformed_pt = target_pose_inverse * pose * pt;
                    compounded_gmm.emplace_back(transformed_pt.x(), transformed_pt.y());
                }
            } else if constexpr (std::is_same_v<T, pcl::PointCloud<pcl::PointXYZ>>) {
                // Process scan data --> a PCL point cloud
                auto pose = computeTransformationMatrix(poses[i]);
                // Convert 3x3 matrices to 4x4 by creating 4x4 identity matrices and inserting the 3x3 matrices in them
                Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
                transformation.block<3, 3>(0, 0) = target_pose_inverse * pose;
                pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
                pcl::transformPointCloud(scan, transformed_cloud, transformation);
                compounded_pcl += transformed_cloud;
            } else if constexpr (std::is_same_v<T, coder::array<double, 2U>>) {
                // Handling for pIC will be done after this loop
                auto pose = computeTransformationMatrix(poses[i]);
                auto transformation = target_pose_inverse * pose;
                for (size_t j = 0; j < scan.size(1); ++j) 
                {
                    Eigen::Vector3d point(scan.at(0, j), scan.at(1, j), 1.0);
                    Eigen::Vector3d transformed_point = transformation.cast<double>() * point;
                    x.push_back(transformed_point.x());
                    y.push_back(transformed_point.y());
                    

                }

            }
           
        }, scans[i]);
    }

    
    // Handle the compounding for pIC
    coder::array<double, 2U> compounded_pic;
    if (x.size() > 0){
        
        compounded_pic.set_size(2, x.size());
        for (size_t j = 0; j < x.size(); ++j) {
            compounded_pic.at(0, j) = x[j];
            compounded_pic.at(1, j) = y[j];
        }
    }



    // Decide on the type to return based on the data processed
    if (!compounded_pcl.empty()) {
        return compounded_pcl;
    } else if (!compounded_gmm.empty()) {
        return compounded_gmm;
    } else if (x.size() > 0) {
        return compounded_pic;
    } else {
        throw std::runtime_error("No valid point cloud data found.");
    }
}


/**
 * Updates the pose and covariance of a robot.
 * This function applies a motion model to propagate the pose and covariance matrix forward in time.
 * 
 * @param Poses A vector of vectors containing the poses (x, y, yaw).
 * @param Covariances A vector of 3x3 Eigen matrices containing the covariances associated with each pose.
 * @param uk The control input in the form of a tuple (delta x, delta y, delta theta).
 * @param new_cov The covariance matrix of the control input, representing motion uncertainty.
 */
void updatePoseAndCovariances(std::vector<std::vector<float>>& Poses, std::vector<Eigen::Matrix3d>& Covariances, 
                const std::tuple<float, float, float>& uk, const Eigen::Matrix3d& new_cov) {
    if (Poses.empty()) return; // Safeguard against empty Poses vector

    // Extract uk components and normalize angle
    float uk_x = std::get<0>(uk);
    float uk_y = std::get<1>(uk);
    float uk_theta = normalize_angle(std::get<2>(uk));

    // Create previous pose transformation matrix
    float cos_theta = cos(Poses.back()[2]);
    float sin_theta = sin(Poses.back()[2]);
    Eigen::Matrix3f previousPose;
    previousPose << cos_theta, -sin_theta, Poses.back()[0],
                    sin_theta,  cos_theta, Poses.back()[1],
                        0.0,         0.0,                1.0;

    // Create the motion transformation matrix using the hat operator
    Eigen::Matrix3f uk_hat;
    uk_hat <<   0,         -uk_theta, uk_x,
                uk_theta,   0.0,      uk_y,
                0.0,        0.0,      0.0;

    // Calculate the new pose
    Eigen::Matrix3f eta_k = Eigen::Matrix3f::Identity() + uk_hat; 
    Eigen::Matrix3f new_pose = previousPose * eta_k;

    // Calculate the new yaw angle using atan2
    float new_yaw = atan2(new_pose(1, 0), new_pose(0, 0));


    // *Propagate the Uncertainty*
    if (not Poses.empty()){
    double cos_theta_double = static_cast<double>(cos_theta);
    double sin_theta_double = static_cast<double>(sin_theta);
    double x = static_cast<double>(Poses.back()[0]);
    double y = static_cast<double>(Poses.back()[1]);
    
    // form the adjoint matrix
    Eigen::Matrix3d Adj;
    Adj << cos_theta_double, -sin_theta_double,  y,
           sin_theta_double,  cos_theta_double, -x,
              0.0,                 0.0,         1.0;

    Eigen::Matrix3d cov = Adj * Covariances.back() * Adj.transpose() + new_cov;

    // Add the new cov to Covariances
    Covariances.push_back(cov);}

    // Add the new pose to Poses
    Poses.push_back({new_pose(0, 2), new_pose(1, 2), new_yaw});
}



/**
 * Publishes the robot's pose as an odometry message using the current estimate.
 * @param pub Reference to the ROS publisher object.
 * @param pose Vector containing the robot's current pose (x, y, yaw).
 */
void publishOdometry(ros::Publisher &pub ,const std::vector<float>& pose) {
    
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "world_ned";

    // Set pose 
    odom.pose.pose.position.x = pose[0];
    odom.pose.pose.position.y = pose[1];
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(pose[2]); 

    // Publish the message
    pub.publish(odom);
}



/**
 * Publishes the result of a scan-matching algorithm with position and orientation uncertainty.
 * @param pub Reference to the ROS publisher object.
 * @param pose_vector Vector containing the pose estimated by scan-matching (x, y, yaw).
 * @param covariance 3x3 covariance matrix for the estimated pose.
 */
void publish_sm_result(ros::Publisher &pub, const std::vector<float> &pose_vector, const Eigen::Matrix3d &covariance)
{
    // Ensure the pose_vector has exactly three elements: x, y, yaw
    if (pose_vector.size() != 3) {
        ROS_ERROR("The pose vector must contain exactly three elements: x, y, and yaw.");
        return;
    }

    // Create and fill the message
    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.frame_id = "world_ned";
    pose_msg.header.stamp = ros::Time::now();

    // Set position
    pose_msg.pose.pose.position.x = pose_vector[0];
    pose_msg.pose.pose.position.y = pose_vector[1];
    pose_msg.pose.pose.position.z = 0;  

    // Set orientation (yaw)
    tf::Quaternion q = tf::createQuaternionFromYaw(pose_vector[2]);
    quaternionTFToMsg(q, pose_msg.pose.pose.orientation);

    // Set covariance, covariance is a 3x3 matrix (x, y, yaw)
    Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> map_cov(pose_msg.pose.covariance.data());
    map_cov.setZero();  // Zero out the full 6x6 matrix
    map_cov.block<2, 2>(0, 0) = covariance.block<2, 2>(0, 0);  // x, y
    map_cov(5, 5) = covariance(2, 2);  // yaw
    map_cov(0, 5) = covariance(0, 2);  // x,yaw
    map_cov(1, 5) = covariance(1, 2);  // y,yaw
    map_cov(5, 0) = covariance(2, 0);  // yaw,x
    map_cov(5, 1) = covariance(2, 1);  // yaw,y

    // Publish the message
    pub.publish(pose_msg);
}


