#include <cmath>
#include "sonar_point_cloud/helper_functions.h"

/**
 * Detects potential loop closure candidates by comparing a new pose with all previous poses using a Mahalanobis distance measure.
 * This function is intended to identify when a robot has revisited a location, which is useful in SLAM (Simultaneous Localization and Mapping).
 *
 * @param poses A vector containing all previously visited poses.
 * @param new_pose The new pose of the robot to compare against previous poses.
 * @param cov The covariance matrix of the new pose's measurement uncertainty.
 * @param confidence_level The statistical confidence level used to determine the threshold for loop closure detection.
 * @return A vector of indices of the poses that are considered as potential loop closure candidates.
 */

std::vector<int> detect_loop_closure(const std::vector<std::vector<float>>& poses, const std::vector<float>& new_pose,
                                     const gtsam::Matrix& cov, const float confidence_level) {
    std::vector<int> indices;

    if (poses.size() < 3) {
        return indices;  // Not enough poses to consider
    }

    // Define the thresholds
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
    
    else {std::cerr << "Warning: Currently 0.68, 0.90, 0.95 and 0.99 confidence_levels are only supported. Please choose a valid confidence_level. Returning {}." << std::endl;
    return indices;}

    // convert gtsam::Matrix to Eigen::Matrix3f to avoid future problems
    Eigen::Matrix3f cov1 = cov.cast<float>(); // cov of the last pose--- source pose
    // Extract the top-left 2x2 submatrix ---- yaw is not relevant here ... we have 360 degree view
    Eigen::Matrix2f submatrix_cov = cov1.block<2,2>(0, 0);

    Eigen::Vector2f dist;

    float last_x = new_pose[0];
    float last_y = new_pose[1];

    for (size_t i = 0; i < poses.size() - 1; ++i){
        float x = poses[i][0];
        float y = poses[i][1];
        dist << std::abs(last_x - x), std::abs(last_y - y);

        // compute the squared mahalanobis distance
        float squared_mahalanobis_dist = dist.transpose() * submatrix_cov.inverse() * dist;

        if (squared_mahalanobis_dist <=  critical_value) {
            indices.push_back(i);
        }
        
    }

    return indices;
  
}


