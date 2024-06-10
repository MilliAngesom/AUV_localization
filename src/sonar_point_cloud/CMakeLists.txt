cmake_minimum_required(VERSION 3.10)
project(sonar_point_cloud)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# for debugging
set(CMAKE_BUILD_TYPE Debug)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  cv_bridge
  tf
  graph_slam_visualizer
  )

find_package(OpenCV REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Development)
find_package(Python3 REQUIRED COMPONENTS Development NumPy)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)


find_package(gmm_registration REQUIRED) 



# Adding the pIC source code
# Add executable target with source files
file(GLOB_RECURSE SOURCE_FILES "${CMAKE_SOURCE_DIR}/src/pIC_registration/*.cpp")


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs nav_msgs visualization_msgs tf
  DEPENDS OpenCV PCL EIGEN3
  geometry_msgs
)


include_directories(
  include  # This should point to the include directory in the package
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
  ${gmm_registration_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}

  # Using environment variable for matplotlib-cpp
  $ENV{MATPLOTLIB_CPP_INCLUDE_DIR}

  
  ${CMAKE_CURRENT_SOURCE_DIR}/include/pIC_registration
  ${CMAKE_CURRENT_SOURCE_DIR}/include/GraphBuilder
  ${catkin_WS}/src/graph_slam_visualizer/include
  

)




link_directories(${PCL_LIBRARY_DIRS})

# Declare executables
add_executable(test_sonar_point_cloud_extractor src/test_sonar_point_cloud_extractor.cpp)
add_executable(fls_slam_node src/fls_slam_node.cpp src/point_cloud_extractor.cpp src/scan_matcher.cpp
                         src/visualisation_publishers.cpp src/GraphBuilder.cpp src/detect_loop_closure.cpp 
                         src/helper_functions.cpp ${SOURCE_FILES}) 

add_executable(msis_slam_node src/msis_slam_node.cpp src/point_cloud_extractor.cpp src/scan_matcher.cpp
                         src/visualisation_publishers.cpp src/GraphBuilder.cpp src/detect_loop_closure.cpp 
                         src/helper_functions.cpp ${SOURCE_FILES}) 
                        
add_executable(v1_msis_slam src/v1_msis_slam.cpp src/point_cloud_extractor.cpp src/scan_matcher.cpp
                         src/visualisation_publishers.cpp src/GraphBuilder.cpp src/detect_loop_closure.cpp 
                         src/helper_functions.cpp ${SOURCE_FILES}) 

add_executable(real_msis_slam_node src/real_msis_slam_node.cpp src/point_cloud_extractor.cpp src/scan_matcher.cpp
                         src/visualisation_publishers.cpp src/GraphBuilder.cpp src/detect_loop_closure.cpp 
                         src/helper_functions.cpp ${SOURCE_FILES}) 

add_executable(slam_node src/slam_node.cpp src/point_cloud_extractor.cpp src/scan_matcher.cpp
                         src/visualisation_publishers.cpp src/GraphBuilder.cpp src/detect_loop_closure.cpp 
                         src/helper_functions.cpp ${SOURCE_FILES}) 

                         
                         
# Link libraries
target_link_libraries(test_sonar_point_cloud_extractor
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Python3_LIBRARIES}
)

target_link_libraries(fls_slam_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Python3_LIBRARIES}
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  gmm_registration
  gomp
  ${GTSAM_LIBRARIES}
  gtsam
)


target_link_libraries(msis_slam_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Python3_LIBRARIES}
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  gmm_registration
  gomp
  ${GTSAM_LIBRARIES}
  gtsam
)




target_link_libraries(v1_msis_slam
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Python3_LIBRARIES}
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  gmm_registration
  gomp
  ${GTSAM_LIBRARIES}
  gtsam
)

target_link_libraries(real_msis_slam_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Python3_LIBRARIES}
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  gmm_registration
  gomp
  ${GTSAM_LIBRARIES}
  gtsam
)


target_link_libraries(slam_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Python3_LIBRARIES}
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  gmm_registration
  gomp
  ${GTSAM_LIBRARIES}
  gtsam
)

