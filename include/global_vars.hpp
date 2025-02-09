#pragma once

#include <Eigen/Dense>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/surface/poisson.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/centroid.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/pcl_visualizer.h>



// Define a structure for a cylinder
struct cylinder_obs
{
    Eigen::Vector3f baseCenter;
    Eigen::Vector3f axis;
    float radius;
    float height;
};

// List of cylinders
extern std::vector<cylinder_obs> cylinders;

// Declare the cloud variable
extern pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

// Declare other global variables
extern bool voxel_grid_initialized;
extern pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
extern pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;