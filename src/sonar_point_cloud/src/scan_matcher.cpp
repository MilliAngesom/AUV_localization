#include "sonar_point_cloud/scan_matcher.h"

// Header for ICP and GICP
#include <pcl/registration/gicp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// // Headers for GMM
#include <gmm_registration/eigen_fix.h>
#include <gmm_registration/front_end/GaussianMixturesModel.h>
#include <gmm_registration/front_end/GmmFrontEnd.hpp>
#include <gmm_registration/method/PointsToDistribution2D.h>
#include <gmm_registration/method/ScanMatchingMethods.h>
#include <gmm_registration/solver/CholeskyLineSearchNewtonMethod.h>
#include <gmm_registration/solver/Solver.h>
#include <time.h>

// Headers for pIC
#include "pIC_registration.h"
#include "pIC_registration_terminate.h"
#include "rt_nonfinite.h"
#include "coder_array.h"

// General Purpose Headers
#include <chrono>
#include <thread>
#include <cmath>
#include <iostream> 
#include <variant>
#include "matplotlibcpp.h"
#include <mutex>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>

// For debugging purpose
#include <matplotlibcpp.h>


// Define PointCloudVariant
using PointCloudVariant = std::variant
    <
        std::vector<Eigen::Vector2d>,             // for gmm
        pcl::PointCloud<pcl::PointXYZ>,           // for ICP and GICP
        coder::array<double, 2U>                  // for pIC
    >;

// Define a variable to store the aligned PC
pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
pcl::PointCloud<pcl::PointXYZ> temp;
std::mutex alignmentMutex;


void visualiseMatchedScansGICP_ICP(const pcl::PointCloud<pcl::PointXYZ>& target_cloud, const pcl::PointCloud<pcl::PointXYZ>& aligned_cloud); // for debugging purpose

/**
 * Registers two point clouds using different algorithms based on the input.
 * Supported algorithms are GICP, ICP, GMM, and pIC. The function calculates the optimal transformation
 * to align the source point cloud to the target point cloud and returns the transformation parameters.
 *
 * @param source Variant containing the source point cloud in one of the supported formats.
 * @param target Variant containing the target point cloud in one of the supported formats.
 * @param initial_guess Vector containing the initial guess for transformation parameters (x, y, yaw).
 * @param initial_guess_cov Vector containing the initial guess covariance for the transformation parameters.
 * @param sonar_noise Noise parameter for the sonar, used in the registration algorithm.
 * @param estimated_registration_cov Reference to an Eigen matrix to store the estimated registration covariance.
 * @param algorithm String specifying the registration algorithm to use (gicp, icp, gmm, pic).
 * @return Tuple containing the transformation parameters (x, y, yaw).
 */

std::tuple<float, float, float> register_scans(const PointCloudVariant& source, const PointCloudVariant& target, 
                                            const std::vector<float>& initial_guess, const std::vector<double> initial_guess_cov, 
                                            double sonar_noise, Eigen::Matrix3d& estimated_registration_cov,
                                            const std::string& algorithm)
{  
    
    // get the exact data type the PointCloudVariant type variable holds
    if (std::holds_alternative<pcl::PointCloud<pcl::PointXYZ>>(source) &&
        std::holds_alternative<pcl::PointCloud<pcl::PointXYZ>>(target) ) 
    {
                
        auto& source_cloud = std::get<pcl::PointCloud<pcl::PointXYZ>>(source);
        auto& target_cloud = std::get<pcl::PointCloud<pcl::PointXYZ>>(target);
        

        // Convert pcl::PointCloud to a smart pointer (necessary for GICP)
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_ptr(new pcl::PointCloud<pcl::PointXYZ>(source_cloud));
        pcl::PointCloud<pcl::PointXYZ>::Ptr target_ptr(new pcl::PointCloud<pcl::PointXYZ>(target_cloud));
        

        if(algorithm=="gicp")
        {
            // Create a dynamic-size matrix and specify its size ............ to model the sensor uncertainty
            Eigen::Matrix3d CovMatrix(3, 3);

            // Initialize the diagonal matrix with zeros
            CovMatrix.setZero();

            // Set different values on the diagonal
            CovMatrix.diagonal() << sonar_noise, sonar_noise,0.000001;

            
            auto source_pc_covariances = boost::make_shared<std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>>>();
            for(int i=0; i < source_ptr->points.size(); ++i)
            {
                source_pc_covariances->push_back(CovMatrix);
            };

            auto target_pc_covariances = boost::make_shared<std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>>>();
            for(int i=0; i < target_ptr->points.size(); ++i)
            {
                target_pc_covariances->push_back(CovMatrix);
            };
            // Initialize the GICP object
            pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
             

            gicp.setInputSource(source_ptr);
            gicp.setInputTarget(target_ptr);

            // Covariances of the source and target point clouds
            gicp.setSourceCovariances(source_pc_covariances);
            gicp.setTargetCovariances(target_pc_covariances);

            // Set GICP parameters
            gicp.setMaximumIterations(10000);                       // Maximum number of iterations
            gicp.setTransformationEpsilon(1e-8);                    // Transformation epsilon (convergence criteria)
            gicp.setEuclideanFitnessEpsilon(1e-5);                  // Euclidean fitness epsilon (convergence criteria)
            gicp.setMaxCorrespondenceDistance(3.5);  //1.5          // Maximum correspondence distance
            gicp.setRANSACOutlierRejectionThreshold(0.5); //0.5     // RANSAC outlier rejection threshold

            // GICP specific parameters
            gicp.setRotationEpsilon(1e-3);                     // Set the rotation epsilon (convergence criteria for rotation)
            gicp.setCorrespondenceRandomness(5);               // Number of neighbors used for covariance estimation
            gicp.setMaximumOptimizerIterations(100000000);     // Maximum number of iterations for the internal optimization


            
            // Lock the mutex before performing the alignment
            Eigen::Matrix4f initial_transformation = createTransformationMatrix(initial_guess);
            alignmentMutex.lock();
            aligned_cloud.clear();

            // if initial guess is provided give it to gicp else use gicp's default initial guess
            if(initial_guess[0]==0.0f && initial_guess[1]==0.0f && initial_guess[2]==0.0f){
            gicp.align(aligned_cloud);}
            else{ gicp.align(aligned_cloud, initial_transformation);}


            alignmentMutex.unlock();

            if (gicp.hasConverged()) 
            {
                // Extract the resulting transformation
                Eigen::Matrix4f final_transform = gicp.getFinalTransformation();

                // Extract translation (x, y) and yaw (rotation around Z)
                float x = final_transform(0, 3);
                float y = final_transform(1, 3);
                float yaw = std::atan2(final_transform(1, 0), final_transform(0, 0));

                // estimated covariance 
                estimated_registration_cov << std::pow(0.2,2), 0.0, 0.0,
                                              0.0, std::pow(0.2,2), 0.0,
                                              0.0, 0.0, std::pow(0.05,2);

                /////////
                /////// FOR debugging purpose

                // Call the function to visualize the initial data
                visualiseMatchedScansGICP_ICP(target_cloud, aligned_cloud);
                
            
                
                ///////////////////
                std::cout << "**********************************************" << std::endl;
                std::cout << "Row GICP result" << " x: "<< x << " y: " << y << " yaw: " << yaw << std::endl;

                std::cout << "estimated_registration_cov \n" << estimated_registration_cov << std::endl;


                return std::make_tuple(x, y, yaw);
            } 
            else 
            {
                std::cerr << "GICP failed to converge." << std::endl;
                return std::make_tuple(0.0f, 0.0f, 0.0f); // Return zero if the alignment fails
            }
        
        }
        else if (algorithm=="icp")
        {
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            
        
            icp.setInputSource(source_ptr);
            icp.setInputTarget(target_ptr);

            // Set ICP parameters
            icp.setTransformationEpsilon(1e-8);
            icp.setEuclideanFitnessEpsilon(1e-8);
            icp.setMaxCorrespondenceDistance(1.5);
            icp.setRANSACOutlierRejectionThreshold(0.5); 
            icp.setMaximumIterations(100000);

            

            // Perform the alignment
            // Lock the mutex before performing the alignment
            Eigen::Matrix4f initial_transformation = createTransformationMatrix(initial_guess);
            alignmentMutex.lock();
            aligned_cloud.clear();
            // if initial guess is provided give it to icp else use icp's default initial guess
            if(initial_guess[0]==0.0f && initial_guess[1]==0.0f && initial_guess[2]==0.0f){
            icp.align(aligned_cloud);}
            else{ icp.align(aligned_cloud, initial_transformation);}
            alignmentMutex.unlock();

            if (icp.hasConverged()) 
            {
                // Extract the resulting transformation
                Eigen::Matrix4f final_transform = icp.getFinalTransformation();

                // Extract translation (x, y) and yaw (rotation around Z)
                float x = final_transform(0, 3);
                float y = final_transform(1, 3);
                float yaw = std::atan2(final_transform(1, 0), final_transform(0, 0));


                // estimated covariance 
                estimated_registration_cov << std::pow(0.2,2), 0.0, 0.0,
                                              0.0, std::pow(0.2,2), 0.0,
                                              0.0, 0.0, std::pow(0.05,2);


                /////////
                /////// FOR debugging purpose

                // Call the function to visualize the initial data
                visualiseMatchedScansGICP_ICP(target_cloud, aligned_cloud);
                
                std::cout << "**********************************************" << std::endl;
                std::cout << "Row ICP result" << " x: "<< x << " y: " << y << " yaw: " << yaw << std::endl;

                std::cout << "estimated_registration_cov \n" << estimated_registration_cov << std::endl;
                ///////////////////

                return std::make_tuple(x, y, yaw);
            } 
            else 
            {
                std::cerr << "ICP failed to converge." << std::endl;
                return std::make_tuple(0.0f, 0.0f, 0.0f); // Return zero if the alignment fails
            }
        }

        

    }

    // FOR GMM
    else if (std::holds_alternative<std::vector<Eigen::Vector2d>>(source) &&
        std::holds_alternative<std::vector<Eigen::Vector2d>>(target) ) 
    {  
        
        alignmentMutex.lock();
        aligned_cloud.clear();
        
        auto& source_cloud = std::get<std::vector<Eigen::Vector2d>>(source);
        auto& target_cloud = std::get<std::vector<Eigen::Vector2d>>(target);

        // Generate a GMM out of the target scan
        shared_ptr<GaussianMixturesModel<2>> gmm;

        //EM front-end
        gmm = em_constructor(target_cloud,4);

        // //GMM front-end
        // gmm = bayesian_gmm_constructor(target_cloud, 20);
        // gmm->balance_covariances(0.9); //0.5
        // // gmm->plot_components_density(0, target_cloud, true);

        // Set both the GMM and the scan to a P2D method
        shared_ptr<PointsToDistribution2D> method(
            new PointsToDistribution2D(gmm, std::make_shared<std::vector<Eigen::Vector2d>>(source_cloud)));
        
        // Set the method to the solver with its needed parameters
        unique_ptr<CholeskyLineSearchNewtonMethod<3>> solver(new CholeskyLineSearchNewtonMethod<3>(method));

         // Solve the registration problem starting with a zero seed
         Eigen::Matrix<double, 3, 1> initial_transformation;
        for (int i = 0; i < 3; ++i) {
            initial_transformation(i, 0) = initial_guess[i];
            }
        // if initial guess is provided give it to gmm else use gmm's default initial guess
        if(initial_guess[0]==0.0f && initial_guess[1]==0.0f && initial_guess[2]==0.0f){
        solver->compute_optimum();}
        else{ solver->compute_optimum(initial_transformation);}

        
        // solver->plot_process(0, true);
        Eigen::Vector3d t_opt = solver->get_optimal();
        // Eigen::Matrix3d h_opt = solver->get_optimal_uncertainty();
        estimated_registration_cov = 1*solver->get_optimal_uncertainty();

        std::cout << "estimated_registration_cov: \n" << estimated_registration_cov << std::endl;
        
        ///////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////
        // For debugging
        
        
        
        // Transform current_scan        
        for (const auto& point : source_cloud) {
            Eigen::Matrix2d R; // Rotation matrix
            R << cos(t_opt(2)), -sin(t_opt(2)),
                sin(t_opt(2)), cos(t_opt(2));
            Eigen::Vector2d t(t_opt(0), t_opt(1)); // Translation vector
            Eigen::Vector2d transformed_point = R * point + t;
            aligned_cloud.push_back(pcl::PointXYZ(transformed_point(0), transformed_point(1), 0));
        }

        pcl::PointCloud<pcl::PointXYZ> temp;
         for (const auto& point : target_cloud) {
            temp.push_back(pcl::PointXYZ(point(0), point(1), 0));
        }
        // target_cloud = temp;
        alignmentMutex.unlock();
        visualiseMatchedScansGICP_ICP(temp, aligned_cloud);

        ///////////////////////////////////////////
        //////////////////////////////////////////
        std::cout << "**********************************************" << std::endl;
        std::cout << "Row GMM result" << " x: "<< t_opt[0] << " y: " << t_opt[1] << " yaw: " << t_opt[2] << std::endl;


        gmm.reset();
        method.reset();
        solver.reset();

        return std::make_tuple(t_opt[0], t_opt[1], t_opt[2]);
    }

    // For pIC
    else if (std::holds_alternative<coder::array<double, 2U>>(source) &&
             std::holds_alternative<coder::array<double, 2U>>(target) )
    {   
        auto& source_cloud = std::get<coder::array<double, 2U>>(source);
        auto& target_cloud = std::get<coder::array<double, 2U>>(target);

        const double initial_estimate[3] = {
            static_cast<double>(initial_guess[0]),
            static_cast<double>(initial_guess[1]),
            static_cast<double>(initial_guess[2])
        };

        const double initial_estimate_cov[3] = {
            initial_guess_cov[0], 
            initial_guess_cov[1],
            initial_guess_cov[2]
        };

        double q[3];      // estimated transformation as a result of the registration process
        double out;      // flag if the solution was found or not; 1-> found and 0-> no solution found
        double num_it;   // number of iteration to complete the scan-matching
        // double registration_uncertainty[9]; // uncertainty of the registration process.  
        double registration_uncertainty[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                                                           

        

        double source_size = source_cloud.size(1);
        double target_size = target_cloud.size(1);

        

        ///////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////
        
        alignmentMutex.lock();
        aligned_cloud.clear();

        pIC_registration(source_cloud,
                 target_cloud, sonar_noise,
                 initial_estimate,
                 initial_estimate_cov, source_size,
                 target_size, &out, q, &num_it,
                 registration_uncertainty);

        

        
        // Fill the matrix
        estimated_registration_cov << 100* registration_uncertainty[0], 100* registration_uncertainty[1], 100* registration_uncertainty[2],
                                      100* registration_uncertainty[3], 100* registration_uncertainty[4], 100* registration_uncertainty[5],
                                      100* registration_uncertainty[6], 100* registration_uncertainty[7], 100* registration_uncertainty[8]; 

        
        
        // For debugging
        // Transform current_scan        
        size_t size = source_cloud.size(1);
        for (size_t i = 0; i < size; ++i)
        {
            Eigen::Vector2d point(source_cloud.at(0,i), source_cloud.at(1,i));
            Eigen::Matrix2d R; // Rotation matrix
            R << cos(q[2]), -sin(q[2]),
                sin(q[2]), cos(q[2]);
            Eigen::Vector2d t(q[0], q[1]); // Translation vector

            Eigen::Vector2d transformed_point = R * point + t;
            aligned_cloud.push_back(pcl::PointXYZ(transformed_point(0), transformed_point(1), 0));
        }

        pcl::PointCloud<pcl::PointXYZ> temp;
        size = target_cloud.size(1); 
        for (size_t i = 0; i < size; ++i)
        {
            temp.push_back(pcl::PointXYZ(target_cloud.at(0,i), target_cloud.at(1,i), 0));
        }
        // target_cloud = temp;
        alignmentMutex.unlock();
        visualiseMatchedScansGICP_ICP(temp, aligned_cloud);
        

        ///////////////////////////////////////////
        //////////////////////////////////////////

        std::cout << "**********************************************" << std::endl;
        std::cout << "Row pIC result" << " x: "<< q[0] << " y: " << q[1] << " yaw: " << q[2] << std::endl;

        return std::make_tuple(q[0], q[1], q[2]);
    }
     


}


/**
 * Function to convert a vector of transformation parameters (x, y, yaw) to a 4x4 transformation matrix.
 * This matrix is useful for applying transformations in 3D space.
 *
 * @param initial_guess Vector containing the initial transformation parameters.
 * @return A 4x4 transformation matrix of type Eigen::Matrix4f.
 */
Eigen::Matrix4f createTransformationMatrix(const std::vector<float>& initial_guess) 
{   

    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity(); // Initialize Identity Matrix

    // Rotation matrix around the Z axis
    Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();
    rotation(0, 0) = cos(initial_guess[2]);
    rotation(0, 1) = -sin(initial_guess[2]);
    rotation(1, 0) = sin(initial_guess[2]);
    rotation(1, 1) = cos(initial_guess[2]);

    // Set the rotation part
    transformation.block<3,3>(0,0) = rotation;

    // Set the translation part
    transformation(0, 3) = initial_guess[0];
    transformation(1, 3) = initial_guess[1];

    return transformation;
}





namespace plt = matplotlibcpp;

/**
 * Visualizes the matched scans using matplotlib for debugging purposes.
 * This function plots the target and aligned point clouds to assess the quality of scan matching.
 *
 * @param target_cloud Point cloud representing the target scan.
 * @param aligned_cloud Point cloud representing the aligned scan after registration.
 */
void visualiseMatchedScansGICP_ICP(const pcl::PointCloud<pcl::PointXYZ>& target_cloud, const pcl::PointCloud<pcl::PointXYZ>& aligned_cloud) {
    static bool first_plot = true; // Tracks whether this is the first call to the function

    // Extract x, y, z coordinates of source cloud and aligned cloud
    std::vector<float> target_x, target_y, target_z;
    std::vector<float> aligned_x, aligned_y, aligned_z;

    for (const auto& point : target_cloud.points) {
        target_x.push_back(point.x);
        target_y.push_back(point.y);
        target_z.push_back(point.z);
    }

    for (const auto& point : aligned_cloud.points) {
        aligned_x.push_back(point.x);
        aligned_y.push_back(point.y);
        aligned_z.push_back(point.z);
    }

    // Check if the data vectors are empty
    if (target_x.empty() || aligned_x.empty()) {
        std::cerr << "Error: Attempted to plot empty data sets." << std::endl;
        return; // Exit the function if data is empty
    }

    if (first_plot) {
        plt::figure();
        plt::xlim(-25, 25);
        plt::ylim(-25, 25);
        plt::title("Scan Matching Result");
        first_plot = false;
    }

    // Update the plot
    try {
        plt::clf();
        plt::scatter(target_x, target_y, 5.0, {{"label", "Target Cloud"}});
        plt::scatter(aligned_x, aligned_y, 5.0, {{"label", "Aligned Cloud"}});
        plt::legend();
        plt::draw();
        plt::pause(0.01);
    } catch (const std::exception& e) {
        std::cerr << "Failed to draw plot: " << e.what() << std::endl;
    }
}


                    
                    
                    
