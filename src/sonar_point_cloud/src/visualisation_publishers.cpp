#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <vector>
#include "sonar_point_cloud/visualisation_publishers.h"
#include <gtsam/geometry/Pose2.h>
#include <tf/tf.h>
#include "graph_slam_visualizer/GraphSE2.h"
#include "graph_slam_visualizer/EdgeSE2.h"
#include "graph_slam_visualizer/VertexSE2.h"
#include "sonar_point_cloud/helper_functions.h"

/**
 * Initializes (if necessary) and publishes new robot positions using ROS markers.
 * This function supports dynamic updates to the trajectory being visualized.
 *
 * @param marker_pub ROS publisher object to publish markers.
 * @param position Current robot position to be added to the trajectory.
 * @param points Marker for displaying points in the trajectory.
 * @param line_strip Marker for displaying the trajectory as a line strip.
 * @param ns Namespace for the markers, used to differentiate between different marker streams.
 * @param initialized Flag indicating whether the markers have been initialized.
 */
void robot_trajectory_publisher(ros::Publisher &marker_pub, const std::vector<float>& position, visualization_msgs::Marker &points, visualization_msgs::Marker &line_strip, const std::string &ns, bool initialized) {
    if (!initialized) {
        // Initialize markers if it's the first call
        points.header.frame_id = line_strip.header.frame_id = "world_ned";
        points.ns = line_strip.ns = ns;
        points.action = line_strip.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
        points.id = 0;
        line_strip.id = 1;
        points.type = visualization_msgs::Marker::POINTS;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        points.scale.x = points.scale.y = 0.2;  // Scale for points
        line_strip.scale.x = 0.05;  // Width for line_strip
        points.color.a = line_strip.color.a = 1.0;  // Alpha for visibility
        if (ns=="ground_truth")
        {
            points.color.g = 1.0;  // Green for points
            line_strip.color.g = 1.0;  // Green for line strip
        }
        else if (ns=="dr")
        {
            points.color.r = 1.0;  // Red for points
            line_strip.color.r = 1.0;  // Red for line strip
        }
        else if (ns=="sm")
        {
            points.color.r = 1.0;  // magenta for points
            points.color.b = 1.0;
            line_strip.color.r = 1.0;  // magenta for line strip
            line_strip.color.b = 1.0; 
        }
        
    }

    // Update timestamp for each marker
    points.header.stamp = line_strip.header.stamp = ros::Time::now();

    // Create the new point
    geometry_msgs::Point p;
    p.x = position[0];
    p.y = position[1];
    p.z = 0;  

    // Add new point to both markers
    points.points.push_back(p);
    line_strip.points.push_back(p);
    


    // Publish both markers
    marker_pub.publish(points);
    marker_pub.publish(line_strip);
}




/**
 * Visualizes an optimized robot trajectory with initialization and dynamic updates.
 * Uses markers to represent trajectory points and the path in ROS.
 *
 * @param marker_pub ROS publisher object to publish markers.
 * @param optimised_poses List of poses representing the optimized trajectory.
 * @param points Marker for displaying points in the trajectory.
 * @param line_strip Marker for displaying the trajectory as a line strip.
 * @param ns Namespace for the markers, differentiating different data streams.
 * @param initialized Flag indicating if the markers have been initialized.
 */
void optimised_robot_trajectory(ros::Publisher &marker_pub, const std::vector<std::vector<float>>& optimised_poses, 
                              visualization_msgs::Marker &points, visualization_msgs::Marker &line_strip, 
                              const std::string &ns, bool initialized) {
    if (!initialized) {
        // Initialize markers if it's the first call
        points.header.frame_id = line_strip.header.frame_id = "world_ned";
        points.ns = line_strip.ns = ns;
        points.action = line_strip.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
        points.id = 0;
        line_strip.id = 1;
        points.type = visualization_msgs::Marker::POINTS;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        points.scale.x = points.scale.y = 0.2;  // Scale for points
        line_strip.scale.x = 0.05;  // Width for line_strip
        points.color.a = line_strip.color.a = 1.0;  // Alpha for visibility
        points.color.r = 0.0;  
        points.color.g = 0.0;
        points.color.b = 1.0;
        line_strip.color.r = 0.0;  
        line_strip.color.g = 0.0;
        line_strip.color.b = 1.0;
        
        
    }

    // Update timestamp for each marker
    points.header.stamp = line_strip.header.stamp = ros::Time::now();

    // Clear previous points
    points.points.clear();
    line_strip.points.clear();

    for (const auto& pose : optimised_poses) {
        geometry_msgs::Point p;
        p.x = pose[0];
        p.y = pose[1];
        p.z = 0;  

        // Add new point to both markers
        points.points.push_back(p);
        line_strip.points.push_back(p);
    }

    // Publish both markers
    marker_pub.publish(points);
    marker_pub.publish(line_strip);
}


/**
 * Publishes covariance markers for a set of poses to visualize the uncertainty of each pose.
 * Each pose is visualized as a sphere with dimensions proportional to the covariance values.
 *
 * @param publisher ROS publisher object to publish marker arrays.
 * @param poses A vector of poses, each represented as a vector of floats (x, y, orientation).
 * @param covariances A vector of Gtsam matrices representing the covariance of each pose.
 */
void optim_publishCovariances( ros::Publisher& publisher, const std::vector<std::vector<float>>& poses,
                                 const std::vector<gtsam::Matrix>& covariances) 
{
    visualization_msgs::MarkerArray markerArray;
    
    for (size_t i = 0; i < poses.size(); ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world_ned";
        marker.header.stamp = ros::Time::now();
        marker.ns = "poses";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = poses[i][0];
        marker.pose.position.y = poses[i][1];
        marker.pose.position.z = 0.0;
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(0*poses[i][2]);

        // Set the scale of the marker, representing the covariance
        //! Don't remove the multiplicative factor 2---- it is used to set the proper sigma, if needed to visualise 2 sigma change std_scale to 2
        int std_scale = 1;
        marker.scale.x = 2 * std_scale * std::sqrt(covariances[i](0, 0)); // Standard deviation in x
        marker.scale.y = 2 * std_scale * std::sqrt(covariances[i](1, 1)); // Standard deviation in y
        marker.scale.z = 0.1; // A small constant for z since we are in 2D

        marker.color.a = 0.3; // Transparency
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        markerArray.markers.push_back(marker);
    }

    publisher.publish(markerArray);
}

/**
 * Publishes covariance markers for a set of poses from scan-matching to visualize uncertainty.
 * Each pose is visualized as a sphere whose dimensions are proportional to the covariance values.
 *
 * @param publisher ROS publisher object to publish marker arrays.
 * @param poses A vector of poses, each represented as a vector of floats (x, y, orientation).
 * @param covariances A vector of Eigen matrices representing the covariance of each pose.
 */
void sm_publishCovariances(ros::Publisher& publisher, const std::vector<std::vector<float>>& poses,
                                   const std::vector<Eigen::Matrix3d>& covariances) 

    {
    visualization_msgs::MarkerArray markerArray;

    for (size_t i = 0; i < poses.size(); ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world_ned";
        marker.header.stamp = ros::Time::now();
        marker.ns = "poses";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = poses[i][0];
        marker.pose.position.y = poses[i][1];
        marker.pose.position.z = 0.0;
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(0*poses[i][2]);

        // Set the scale of the marker, representing the covariance
        //! Don't remove the multiplicative factor 2---- it is used to set the proper sigma, if need to visualise 2 sigma change std_scale to 2
        int std_scale = 1;
        marker.scale.x = 2 * std_scale * std::sqrt(covariances[i](0, 0)); // Standard deviation in x  
        marker.scale.y = 2 * std_scale * std::sqrt(covariances[i](1, 1)); // Standard deviation in y
        marker.scale.z = 0.01; // A small constant for z since we are in 2D

        marker.color.a = 0.3; // Transparency
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;

        markerArray.markers.push_back(marker);
    }

    publisher.publish(markerArray);
}



/**
 * Publishes a visualization of the entire pose graph, including all vertices (poses) and edges (constraints) 
 * to a ROS topic. This function is useful for visualizing the structure of a graph-based SLAM system, helping 
 * in debugging and understanding the graph connectivity and layout.
 *
 * @param pub ROS publisher object used to publish the pose graph visualization.
 * @param graph The nonlinear factor graph from GTSAM representing the SLAM problem.
 * @param initialEstimates The pose of the graph.
 */

void publishGraph(ros::Publisher &pub, const gtsam::NonlinearFactorGraph &graph, const gtsam::Values &initialEstimates) {
    graph_slam_visualizer::GraphSE2 graph_msg;
    graph_msg.header.stamp = ros::Time::now();
    graph_msg.header.frame_id = "world_ned"; 

    // Add vertices
    for (const auto& key_value : initialEstimates) {
        auto symbol = gtsam::Symbol(key_value.key);
        if (symbol.chr() == 'x') {
            graph_slam_visualizer::VertexSE2 vertex;
            auto pose = key_value.value.cast<gtsam::Pose2>();
            vertex.x = pose.x();
            vertex.y = pose.y();
            vertex.theta = pose.theta();
            graph_msg.vertices.push_back(vertex);
        }
    }

    // Add edges
    for (size_t i = 0; i < graph.size(); ++i) {
        auto factor = std::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose2>>(graph[i]);
        if (factor) {
            graph_slam_visualizer::EdgeSE2 edge;
            edge.vi_idx = gtsam::Symbol(factor->key1()).index();
            edge.vj_idx = gtsam::Symbol(factor->key2()).index();
            graph_msg.edges.push_back(edge);
            graph_msg.edge_weights.push_back(1.0); // Change the weight to change the brightness level of the edges
        }
    }

    // Publish the graph message
    pub.publish(graph_msg);
}


// Plot the final point clouds as map of the scanned environment

#include <vector>
#include <variant>
#include <Eigen/Dense>
#include <cmath>
#include "matplotlibcpp.h"
#include <fstream>
#include <iomanip>

namespace plt = matplotlibcpp;

/**
 * Transforms and plots point cloud data according to given ground truth and estimated transformations.
 * This function is designed to assist in visualizing the accuracy of transformation estimations.
 *
 * @param scans A vector containing various types of point clouds (Eigen, PCL, or custom arrays).
 * @param gt_transformations A vector of ground truth transformations (x, y, yaw).
 * @param estimated_transformations A vector of estimated transformations (x, y, yaw).
 * @param method_txt Filename to save the estimated transformations for further analysis.
 */

void plotMap(
    const std::vector<PointCloudVariant>& scans,
    const std::vector<std::vector<float>>& gt_transformations,
    const std::vector<std::vector<float>>& estimated_transformations,
    const std::string& method_txt
) {
    std::vector<double> gt_x_coords;
    std::vector<double> gt_y_coords;
    std::vector<double> est_x_coords;
    std::vector<double> est_y_coords;

    for (size_t i = 0; i < scans.size(); i++) {
        // Apply ground truth transformations
        if (i < gt_transformations.size()) {
            float transX = gt_transformations[i][0];
            float transY = gt_transformations[i][1];
            float yaw = gt_transformations[i][2];
            float cosYaw = std::cos(yaw);
            float sinYaw = std::sin(yaw);

            std::visit([&, cosYaw, sinYaw](auto&& arg) {
                using T = std::decay_t<decltype(arg)>;
                if constexpr (std::is_same_v<T, std::vector<Eigen::Vector2d>>) {
                    for (const auto& point : arg) {
                        gt_x_coords.push_back(point.x() * cosYaw - point.y() * sinYaw + transX);
                        gt_y_coords.push_back(point.x() * sinYaw + point.y() * cosYaw + transY);
                    }
                } else if constexpr (std::is_same_v<T, pcl::PointCloud<pcl::PointXYZ>>) {
                    for (const auto& point : arg.points) {
                        gt_x_coords.push_back(point.x * cosYaw - point.y * sinYaw + transX);
                        gt_y_coords.push_back(point.x * sinYaw + point.y * cosYaw + transY);
                    }
                } else if constexpr (std::is_same_v<T, coder::array<double, 2U>>) {
                    size_t size = arg.size(1); // Assuming size(1) returns the number of columns
                    for (size_t j = 0; j < size; ++j) {
                        gt_x_coords.push_back(arg.at(0, j) * cosYaw - arg.at(1, j) * sinYaw + transX);
                        gt_y_coords.push_back(arg.at(0, j) * sinYaw + arg.at(1, j) * cosYaw + transY);
                    }
                }
            }, scans[i]);
        }

        // Apply estimated transformations
        if (i < estimated_transformations.size()) {
            float transX = estimated_transformations[i][0];
            float transY = estimated_transformations[i][1];
            float yaw = estimated_transformations[i][2];
            float cosYaw = std::cos(yaw);
            float sinYaw = std::sin(yaw);

            std::visit([&, cosYaw, sinYaw](auto&& arg) {
                using T = std::decay_t<decltype(arg)>;
                if constexpr (std::is_same_v<T, std::vector<Eigen::Vector2d>>) {
                    for (const auto& point : arg) {
                        est_x_coords.push_back(point.x() * cosYaw - point.y() * sinYaw + transX);
                        est_y_coords.push_back(point.x() * sinYaw + point.y() * cosYaw + transY);
                    }
                } else if constexpr (std::is_same_v<T, pcl::PointCloud<pcl::PointXYZ>>) {
                    for (const auto& point : arg.points) {
                        est_x_coords.push_back(point.x * cosYaw - point.y * sinYaw + transX);
                        est_y_coords.push_back(point.x * sinYaw + point.y * cosYaw + transY);
                    }
                } else if constexpr (std::is_same_v<T, coder::array<double, 2U>>) {
                    size_t size = arg.size(1); // Assuming size(1) returns the number of columns
                    for (size_t j = 0; j < size; ++j) {
                        est_x_coords.push_back(arg.at(0, j) * cosYaw - arg.at(1, j) * sinYaw + transX);
                        est_y_coords.push_back(arg.at(0, j) * sinYaw + arg.at(1, j) * cosYaw + transY);
                    }
                }
            }, scans[i]);
        }
    }
    
    // Save coordinates to a CSV file
    {std::ofstream file("ground_truth_map");

    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << "ground_truth_map" << std::endl;
        return;
    }

    file << std::fixed << std::setprecision(6); // Set the precision for floating point numbers

    // Write header
    file << "gt_x,gt_y\n";

    // Write data
    size_t size = std::max(gt_x_coords.size(), gt_y_coords.size());
    for (size_t i = 0; i < size; ++i) {
        if (i < gt_x_coords.size()) file << gt_x_coords[i]; else file << "";
        file << ",";
        if (i < gt_y_coords.size()) file << gt_y_coords[i]; else file << "";
        file << "\n";
    }

    file.close();}

   { std::ofstream file(method_txt);

    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << method_txt << std::endl;
        return;
    }

    file << std::fixed << std::setprecision(6); // Set the precision for floating point numbers

    // Write header
    file << "est_x,est_y\n";

    // Write data
    size_t size = std::max(est_x_coords.size(), est_y_coords.size());
    for (size_t i = 0; i < size; ++i) {
        if (i < est_x_coords.size()) file << est_x_coords[i]; else file << "";
        file << ",";
        if (i < est_y_coords.size()) file << est_y_coords[i]; else file << "";
        file << "\n";
    }

    file.close();}

    // Store the ground truth poses
    std::string filename = "Ground_Truth_poses";
    {std::ofstream file("Ground_Truth_poses");
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    file << std::fixed << std::setprecision(6); // Set the precision for floating point numbers
  

    for (const auto& transform : gt_transformations) {
        if (transform.size() == 3) {
            file << transform[0] << " " << transform[1] << " " << transform[2] << "\n";
        } else {
            std::cerr << "Unexpected transformation size" << std::endl;
        }
    }

    file.close();}

    // Store the estimated poses
    filename = method_txt+" poses";
    {std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    file << std::fixed << std::setprecision(6); // Set the precision for floating point numbers
   

    for (const auto& transform : estimated_transformations) {
        if (transform.size() == 3) {
            file << transform[0] << " " << transform[1] << " " << transform[2] << "\n";
        } else {
            std::cerr << "Unexpected transformation size" << std::endl;
        }
    }

    file.close();}

    // // Plotting both ground truth and estimated points
    // plt::figure();
    // plt::figure_size(1200, 800);
    // plt::scatter(gt_x_coords, gt_y_coords, 5.0, {{"label", "Ground Truth"}});
    // plt::scatter(est_x_coords, est_y_coords, 5.0, {{"label", method_txt}});
    // plt::xlabel("X Coordinate");
    // plt::ylabel("Y Coordinate");
    // plt::title("Ground Truth Map vs" + method_txt );
    // plt::legend();
    // plt::show();
}
