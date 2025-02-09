#pragma once

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/PointStamped.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/package.h>  // Include the ROS package utility
#include "ompl/base/SpaceInformation.h"
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <algorithm> // for std::clamp
#include <fstream>


#include <fcl/fcl.h>

#include <fcl/geometry/shape/shape_base-inl.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <fcl/geometry/shape/convex.h>


#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <ompl/geometric/SimpleSetup.h>

#include <fcl/fcl.h>
//#include <fcl/geometry/shape/mesh.h>


#include <vector>
#include <cmath>
#include <iostream>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>

#include <sensor_msgs/PointCloud2.h>
#include <ompl/base/spaces/SE3StateSpace.h>  // Include SE3StateSpace
#include <pcl/kdtree/kdtree_flann.h>

#include <publishers.hpp>
#include <helper_functions.hpp>
#include <pcl/filters/voxel_grid.h>


#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>



#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/surface/poisson.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/centroid.h>
//#include "globals.hpp" // Include the globals.hpp header


#include <my_motion_validator.hpp>

#include "tether_planner.hpp"



extern pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

extern pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
