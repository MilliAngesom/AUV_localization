// point_cloud_extractor.h
#ifndef POINT_CLOUD_EXTRACTOR_H
#define POINT_CLOUD_EXTRACTOR_H

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <variant>
#include <vector>
#include <string>
#include <omp.h>
#include "coder_array.h"



// PointCloudVariant
using PointCloudVariant = std::variant
    <
        std::vector<Eigen::Vector2d>,             // for gmm
        pcl::PointCloud<pcl::PointXYZ>,           // for ICP and GICP
        coder::array<double, 2U>                  // for pIC
    >;

// Function declaration
void polarToCartesian(cv::Mat& raw_image, const float& radius, const float& horizontal_FOV, const uint8_t& intensity_threshold, 
                      const float& min_sonar_range, std::vector<float>& x, std::vector<float>& y, PointCloudVariant& cloud, 
                      int& num_points_per_beam, const std::string& algorithm, const float& sonar_roll,
                      const float& sonar_x_position, const float& sonar_y_position);

bool isPointCloudEmpty();

#endif // POINT_CLOUD_EXTRACTOR_H
