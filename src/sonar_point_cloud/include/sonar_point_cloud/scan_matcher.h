#ifndef SCAN_MATCHER_H
#define SCAN_MATCHER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <variant>
#include <vector>
#include <string>
#include "coder_array.h"



// // Headers for GMM
#include <gmm_registration/eigen_fix.h>
#include <Eigen/Dense>
#include <time.h>
#include <gmm_registration/front_end/GaussianMixturesModel.h>
#include <gmm_registration/front_end/GmmFrontEnd.hpp>
#include <gmm_registration/method/PointsToDistribution2D.h>
#include <gmm_registration/method/ScanMatchingMethods.h>
#include <gmm_registration/solver/CholeskyLineSearchNewtonMethod.h>
#include <gmm_registration/solver/Solver.h>




// Define PointCloudVariant
using PointCloudVariant = std::variant
    <
        std::vector<Eigen::Vector2d>,             // for gmm
        pcl::PointCloud<pcl::PointXYZ>,           // for ICP and GICP
        coder::array<double, 2U>                  // for pIC
    >;

// Function declaration
std::tuple<float, float, float> register_scans(const PointCloudVariant& source, const PointCloudVariant& target, 
                                            const std::vector<float>& initial_guess, const std::vector<double> initial_guess_cov, 
                                            double sonar_noise, Eigen::Matrix3d& estimated_registration_cov,
                                            const std::string& algorithm);

Eigen::Matrix4f createTransformationMatrix(const std::vector<float>& initial_guess); 


#endif // SCAN_MATCHER_H