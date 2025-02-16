#include "global_vars.hpp"

std::vector<cylinder_obs> cylinders;

// Define and initialize the cloud variable
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

// Define other global variables
bool voxel_grid_initialized = false;
pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;


//params
double time_step;


//tetehr
double L_max;


std_msgs::ColorRGBA tetherColor;
std_msgs::ColorRGBA directPath;
std_msgs::ColorRGBA safePath;
std_msgs::ColorRGBA ropepathColor;
std_msgs::ColorRGBA rovpathColor;




//time
std::chrono::duration<double> time_tether_model_computation;




bool reset_tether;
bool record_trajectory;
std::string trajectory_filename;
bool inspection_done = false;
