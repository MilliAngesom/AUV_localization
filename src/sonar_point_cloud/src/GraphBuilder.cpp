#include "GraphBuilder.h"
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Pose2.h>
#include <fstream>
#include <optional>

/**
 * Constructor for the GraphBuilder class.
 * Initializes the graph with a starting node representing the robot's initial pose at the origin
 * and sets up the graph with appropriate priors and uncertainty models.
 */
GraphBuilder::GraphBuilder() : currentIndex(1) {
    // Insert the initial pose to the initialEstimates with key 'x1'
    initialEstimates.insert(gtsam::Symbol('x', currentIndex), gtsam::Pose2(0, 0, 0));
    // Add a prior factor to the graph to anchor the first pose at the origin with a small uncertainty
    graph.add(gtsam::PriorFactor<gtsam::Pose2>(gtsam::Symbol('x', currentIndex), gtsam::Pose2(0, 0, 0), gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.01, 0.01, 0.001))));
}

/**
 * Adds a new pose and an associated BetweenFactor to the graph.
 * The BetweenFactor models the relative transformation (odometry) between the current pose and the new pose,
 * incorporating a noise model to account for the uncertainty of the transformation.
 *
 * @param newPose The new pose to add, expressed as a gtsam::Pose2.
 * @param odometry The relative transformation from the current pose to the new pose, also expressed as a gtsam::Pose2.
 * @param noiseModel The noise model associated with the odometry measurement, encapsulating uncertainty.
 */
void GraphBuilder::addPose(const gtsam::Pose2& newPose, const gtsam::Pose2& odometry, const gtsam::noiseModel::Base::shared_ptr& noiseModel) {
    int newIndex = currentIndex + 1; // Compute the new pose index
    // Insert the new pose into the estimates map
    initialEstimates.insert(gtsam::Symbol('x', newIndex), newPose);
    // Add a between factor to the graph using the provided odometry and noise model
     graph.add(gtsam::BetweenFactor<gtsam::Pose2>(
          gtsam::Symbol('x', currentIndex), gtsam::Symbol('x', newIndex), odometry,  noiseModel));
    // Update the current index to the new index
    currentIndex = newIndex; 
}

/**
 * Adds a loop closure constraint to the factor graph.
 * This method uses a between factor to represent the loop closure,
 * which helps to correct drift accumulated over time in SLAM applications.
 * 
 * @param fromIndex The index of the pose from which the loop starts.
 * @param toIndex The index of the pose where the loop closes.
 * @param measurement The relative pose measurement between the two poses.
 * @param noiseModel The uncertainty model associated with the loop closure measurement.
 */
void GraphBuilder::addLoopClosure(int fromIndex, int toIndex, const gtsam::Pose2& measurement, const gtsam::noiseModel::Base::shared_ptr& noiseModel) {
    // Add a between factor representing the loop closure
    graph.add(gtsam::BetweenFactor<gtsam::Pose2>(
        gtsam::Symbol('x', fromIndex), gtsam::Symbol('x', toIndex), measurement, noiseModel));
}

/**
 * Optimizes the graph using the Levenberg-Marquardt algorithm.
 * This method adjusts the poses in the graph to minimize the overall error,
 * a process known as graph optimization in SLAM.
 */
void GraphBuilder::optimizeGraph() { 
    // Setup optimization parameters    
    Params.setMaxIterations(1000000); 
    Params.setRelativeErrorTol(1e-5); 
    Params.setAbsoluteErrorTol(1e-7);
    // Perform optimization and update the initial estimates
    gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initialEstimates,Params).optimizeSafely();    
    initialEstimates = result; // Update the estimates post optimization
}

/**
 * Prints the current pose estimates stored in the graph.
 * This is useful for debugging and visualization purposes.
 */
void GraphBuilder::printCurrentEstimates() const {
    initialEstimates.print("Current estimates:\n");

    std::cout << "Graph contains the following factors:" << std::endl;
    for (size_t i = 0; i < graph.size(); ++i) {
        std::cout << "Factor " << i << ": ";
        graph.at(i)->print();
        std::cout << std::endl;
    }
}

/**
 * Updates the provided vector with the latest pose estimates from the graph.
 *
 * @param old_poses Vector to be updated with the latest poses.
 */
void GraphBuilder::update_poses(std::vector<std::vector<float>>& old_poses) {
    old_poses.clear();  // Clear existing poses
    for (const auto& key_value : initialEstimates) {
        auto symbol = gtsam::Symbol(key_value.key);
        if (symbol.chr() == 'x') {  // Ensure we are dealing with pose symbols
            auto pose = key_value.value.cast<gtsam::Pose2>();
            std::vector<float> updated_pose = {float(pose.x()), float(pose.y()), float(pose.theta())};
            old_poses.push_back(updated_pose);
        }
    }
}

/**
 * Retrieves the marginal covariances for all poses or a specific pose.
 *
 * @param index Optional index to specify a single pose for which to retrieve the covariance.
 * @return A vector of covariance matrices for the specified pose(s).
 */
std::vector<gtsam::Matrix> GraphBuilder::getMarginalCovariances(std::optional<int> index ) const {
    std::vector<gtsam::Matrix> covariances;
    gtsam::Marginals marginals(graph, initialEstimates); // Compute marginals for the graph

    if (index) {  // If index is provided
        if (*index >= 1 && *index <= currentIndex) {  // Check if the index is within bounds
            // Get the marginal covariance for the specified index
            gtsam::Matrix covariance = marginals.marginalCovariance(gtsam::Symbol('x', *index));
            covariances.push_back(covariance);
        }
    } else {  // If no index is provided, return all covariances
        for (int i = 1; i <= currentIndex; ++i) {
            gtsam::Matrix covariance = marginals.marginalCovariance(gtsam::Symbol('x', i));
            covariances.push_back(covariance);
        }
    }

    return covariances;
}


