# AUV Localization Using MSIS Sonar Scanner

## Overview
This project aims to localize Autonomous Underwater Vehicles (AUVs) using an MSIS sonar scanner within a Pose Graph SLAM framework. It utilizes the GTSAM library to handle the graph operations and incorporates four scan matching algorithms: ICP, GICP, GMM, and pIC. The Girona500 AUV model is used for simulation purposes.

## Prerequisites
Before running this project, ensure the following prerequisites are met:
- **ROS Neotic**: Ensure ROS Neotic is installed.
    ```bash
    export CATKIN_WS=~/catkin_ws/  # Set your Catkin workspace environment variable appropriately.
    ```
- **OpenCV 4.8.0 or Later**: Required for image processing functionalities.
- **C++17 Standard**: Ensure your compiler supports C++17.
- **Point Cloud Library (PCL) 1.10.0**: Can be installed standalone or via ROS.
- **GMM Registration**: If using GMM, install from [GMM Bitbucket](https://bitbucket.org/gmmregistration/gmm_registration/src/master/).
- **Stonefish Simulator**: Install Stonefish for simulation capabilities. Follow instructions at [Stonefish Documentation](https://stonefish.readthedocs.io/en/latest/install.html). After installation, clone and compile the ROS interface:
    ```bash
    git clone https://github.com/patrykcieslak/stonefish_ros ~/catkin_ws/src/
    ```
- **Matplotlib-cpp**: For visualization, set the environment variable for the include directory:
    ```bash
    export MATPLOTLIB_CPP_INCLUDE_DIR=~/matplotlib-cpp --> you have to replace with the correct path where you have saved matplotlib-cpp
    ```
- **Additional Dependencies**: Follow installation guides for additional required packages:
  - [Sonar SLAM](https://bitbucket.org/udg_cirs/sonar_slam)
  - [COLA2 Framework](https://bitbucket.org/iquarobotics/cola2_wiki/src/master/basic_installation.md)
  - [Girona500 Description](https://bitbucket.org/udg_cirs/girona500_description/src/master/)
  - [Stonefish Translator Package](https://bitbucket.org/udg_cirs/cola2_stonefish/src/master/)

## Installation
To set up and run the project, follow these steps:

1. **Clone the Repository**:
    ```bash
    cd ~/catkin_ws/src
    git clone https://github.com/MilliAngesom/AUV_localization.git
    ```

2. **Build the Project**:
    Navigate to the cloned directory, then move the required ROS packages into your `src` folder:
    ```bash
    mv AUV_localization/src/{graph_slam_visualizer,sonar_point_cloud} ~/catkin_ws/src/
    cd ~/catkin_ws
    catkin build
    ```

## Usage
- To launch the simulation, run:
    ```bash
    roslaunch sonar_slam sonar_slam.launch
    ```
- To start the SLAM node:
    ```bash
    rosrun sonar_point_cloud slam_node
    ```
- To enable the thrusters and publish velocity commands:
    ```bash
    rosservice call /girona500/controller/enable_thrusters
    rostopic pub /girona500/controller/body_velocity_req your_message_type '{stamp: now, frame_id: "world_ned", requester: "test", priority: 30}'
    ```
