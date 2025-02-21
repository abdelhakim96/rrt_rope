#pragma once

#include <Eigen/Dense>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <std_msgs/Bool.h>

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
#include <std_msgs/ColorRGBA.h>  // Include the correct header



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
extern std::vector<cylinder_obs> cylinders_safe;


// Declare the cloud variable
extern pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

// Declare other global variables
extern bool voxel_grid_initialized;
extern pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
extern pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;


//params

extern double time_step;

//tether
extern double L_max;


// Define the colors
extern std_msgs::ColorRGBA tetherColor;
extern std_msgs::ColorRGBA directPath;
extern std_msgs::ColorRGBA safePath;
extern std_msgs::ColorRGBA ropepathColor;
extern std_msgs::ColorRGBA rovpathColor;
extern std_msgs::ColorRGBA safepathColor;


extern std::chrono::duration<double> time_tether_model_computation;



extern bool reset_tether;
extern bool record_trajectory;
extern std::string trajectory_filename;
extern bool inspection_done;
extern bool TA_Planner_ON;

extern double delta ;                // Step size
extern double equivalenceTolerance;  // Equivalence tolerance

extern bool goal_reached;


extern bool Tether_Length_exceeded;

//planner params
extern double safe_offset;



extern std::vector<double> way_point;



extern int count ;