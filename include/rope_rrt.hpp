#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Wrench.h>
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

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>


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
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <ompl/base/spaces/SE3StateSpace.h>  // Include SE3StateSpace
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <publishers.hpp>


extern std::vector<double> current_att_quat;
extern std::vector<double> current_vel_rate;
extern std::vector<double> current_pos_att;
extern bool new_data_received;
extern std::vector<double> current_vel_body;
extern std::vector<double> angles;
extern std::vector<double> angles_d;

extern pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

extern pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
