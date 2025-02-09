#include "global_vars.hpp"

std::vector<cylinder_obs> cylinders;

// Define and initialize the cloud variable
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

// Define other global variables
bool voxel_grid_initialized = false;
pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;