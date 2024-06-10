#ifndef GRAPH_BUILDER_H
#define GRAPH_BUILDER_H

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <optional>

#include <vector>

class GraphBuilder {
public:
    GraphBuilder();
    void addPose(const gtsam::Pose2& newPose, const gtsam::Pose2& odometry, const gtsam::noiseModel::Base::shared_ptr& noiseModel);
    void addLoopClosure(int fromIndex, int toIndex, const gtsam::Pose2& measurement, const gtsam::noiseModel::Base::shared_ptr& noiseModel);
    void optimizeGraph();
    void printCurrentEstimates() const;
    void update_poses(std::vector<std::vector<float>>& old_poses); // to update the poses of the robot with the optimised poses from the graph
    std::vector<gtsam::Matrix> getMarginalCovariances(std::optional<int> index = std::nullopt) const; // Method to get marginal covariances
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initialEstimates;
    int currentIndex;

private:
    
    
    gtsam::Ordering ordering;
    gtsam::LevenbergMarquardtParams Params;
    gtsam::GaussNewtonParams parameters;
  
 
  
};

#endif // GRAPH_BUILDER_H
