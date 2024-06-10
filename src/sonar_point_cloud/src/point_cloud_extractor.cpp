#include "sonar_point_cloud/point_cloud_extractor.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <variant>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <omp.h>
#include "coder_array.h"


// Define PointCloudVariant
using PointCloudVariant = std::variant
    <
        std::vector<Eigen::Vector2d>,             // data type for gmm
        pcl::PointCloud<pcl::PointXYZ>,           // data type for ICP and GICP
        coder::array<double, 2U>                  // data type for pIC
    >;

bool IsPointCloudEmpty = true;

/**
 * Converts polar sonar data into Cartesian coordinates and filters points based on intensity and algorithm requirements.
 * This function is critical in sonar image processing, where the sonar data in polar format is transformed into Cartesian coordinates.
 *
 * @param raw_image OpenCV matrix containing the sonar image in polar format.
 * @param radius The maximum range of the sonar in meters.
 * @param horizontal_FOV The horizontal field of view of the sonar in degrees.
 * @param intensity_threshold The minimum intensity value to consider a point valid.
 * @param min_sonar_range The minimum effective range of the sonar in meters.
 * @param x Reference to a vector of floats to store the x-coordinates of detected points.
 * @param y Reference to a vector of floats to store the y-coordinates of detected points.
 * @param cloud A variant type that can hold different types of point clouds.
 * @param num_points_per_beam Number of points to extract per beam, affects the resolution of the output point cloud.
 * @param algorithm Specifies the different algorithms (e.g., GICP, GMM, pIC).
 * @param sonar_roll The roll angle of the sonar in radians, used to adjust the point cloud orientation.
 * @param sonar_x_position X-coordinate of the sonar device, used for translation adjustment.
 * @param sonar_y_position Y-coordinate of the sonar device, used for translation adjustment.
 */
void polarToCartesian(cv::Mat& raw_image, const float& radius, const float& horizontal_FOV, const uint8_t& intensity_threshold, 
                      const float& min_sonar_range, std::vector<float>& x, std::vector<float>& y, PointCloudVariant& cloud, 
                      int& num_points_per_beam, const std::string& algorithm, const float& sonar_roll,
                      const float& sonar_x_position, const float& sonar_y_position) {



    // Check if num_points_per_beam is within a valid range
        if (num_points_per_beam <= 0 || num_points_per_beam > raw_image.rows) {
            std::cerr << "Warning: num_points_per_beam is out of the valid range, default value will be used!" << std::endl;
            num_points_per_beam = 1;
            }

        // Initialize a vector with zeros where the length is equal to the number of columns in raw_image
        std::vector<uchar> intensities(raw_image.cols, 0);

        // non-maxima supression
        #pragma omp parallel for
        // Iterate over each column
        for (int col = 0; col < raw_image.cols; ++col) 
        {
            std::vector<uchar> pixel_intensities;
            pixel_intensities.reserve(raw_image.rows); 

            for (int row = 0; row < raw_image.rows; ++row) {
                pixel_intensities.emplace_back(raw_image.at<uchar>(row, col));
            }

            // Sort pixel values to get the n-th brightest pixel value
            std::nth_element(pixel_intensities.begin(),
                            pixel_intensities.begin() + num_points_per_beam - 1,
                            pixel_intensities.end(), std::greater<uchar>());

            // Access the n-th brightest pixel intensity directly
            intensities[col] = pixel_intensities[num_points_per_beam - 1];
            
        }

    // Initialize cloud to the appropriate type based on the algorithm
    if (algorithm == "gicp" || algorithm == "icp") 
    {   
        cloud = pcl::PointCloud<pcl::PointXYZ>(); // Initialize with pcl::PointCloud
    } 
    else if (algorithm == "gmm") 
    {   
        cloud = std::vector<Eigen::Vector2d>(); // Initialize with std::vector<Eigen::Vector2d>
    }
    else if (algorithm == "pic") 
    {   
        cloud = coder::array<double, 2U>(); // Initialize data type compatible with pIC
    }

    int height = raw_image.rows;
    int width = raw_image.cols;
    float fov_per_pixel = horizontal_FOV / width;
    float deg_to_rad = CV_PI / 180.0f;
    
    for (int i = 0; i < height; ++i) {
        float r = static_cast<float>(height - i) * ((radius- min_sonar_range) / static_cast<float>(height)) + min_sonar_range;
        for (int j = 0; j < width; ++j) {
            uint8_t intensity = raw_image.at<uint8_t>(i, j);
  
            if (intensity >= intensities[j] && intensity >= intensity_threshold) {
                float angle = -0.5 * horizontal_FOV + fov_per_pixel * (j + 0.5f);
                angle = angle * deg_to_rad; // Convert to radians

                // Used For Simulation
                float x_ = r * std::cos(sonar_roll)*std::cos(angle) + sonar_x_position;
                float y_ = r * std::cos(sonar_roll)*std::sin(angle) + sonar_y_position;

                // // For the bag file
                // float x_ = r * std::cos(sonar_roll)*std::cos(angle) + sonar_x_position;
                // float y_ = -r * std::cos(sonar_roll)*std::sin(angle) + sonar_y_position;


                x.push_back(x_);
                y.push_back(y_);

                // Add points to the correct type
                if (std::holds_alternative<pcl::PointCloud<pcl::PointXYZ>>(cloud)) //(algorithm == "gicp" || algorithm == "icp") 
                {
                    auto& pclCloud = std::get<pcl::PointCloud<pcl::PointXYZ>>(cloud);
                    pclCloud.push_back(pcl::PointXYZ(x_, y_, 1));

                } 

                else if (std::holds_alternative<std::vector<Eigen::Vector2d>>(cloud)) //(algorithm == "gmm") 
                {   
                    auto& eigenCloud = std::get<std::vector<Eigen::Vector2d>>(cloud);
                    eigenCloud.emplace_back(x_, y_);
                }              
            }
        }
    }

    // Handle specific case for pIC where we cannot dynamically increase the size of coder::array
    if (std::holds_alternative<coder::array<double, 2U>>(cloud))   //(algorithm == "pic") 
        {
            
            
            auto& picArray = std::get<coder::array<double, 2U>>(cloud);
            // Allocate memory for the array
            picArray.set_size(2, x.size());

            // Fill data
            
            for (size_t j = 0; j < x.size(); ++j) {
                picArray.at(0, j) = static_cast<double>(x[j]); 
                picArray.at(1, j) = static_cast<double>(y[j]);
            }
            
        }
        IsPointCloudEmpty = x.empty();
}



/**
 * Function to check if a valid point cloud has been extracted.
 * @return True if no valid points were extracted, false otherwise.
 */
bool isPointCloudEmpty() {
    return IsPointCloudEmpty;
}