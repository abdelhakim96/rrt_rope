// filepath: /home/hakim/tether_planning_ws/src/rope_rrt/include/publishers.h
#ifndef PUBLISHERS_H
#define PUBLISHERS_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ompl/geometric/PathGeometric.h>
#include <std_msgs/ColorRGBA.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

void publishTetherPath(ros::Publisher &pub, const ompl::geometric::PathGeometric &path, const std::string &frame_id);
void publishPointCloud(ros::Publisher &pub, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
void publishVoxelGrid(ros::Publisher &pub, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
void publishPath(ros::Publisher &pub, const ompl::geometric::PathGeometric &path, const std::string &frame_id, const std::string &ns, const std_msgs::ColorRGBA &color);
//void publishObstacles(ros::Publisher &obstacle_pub, const std::vector<Obstacle> &obstacles, const std::string &frame_id);

#endif // PUBLISHERS_H