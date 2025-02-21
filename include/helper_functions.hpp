#ifndef HELPER_FUNCTIONS_HPP
#define HELPER_FUNCTIONS_HPP


// filepath: /home/hakim/tether_planning_ws/src/rope_rrt/include/publishers.h
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
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PointStamped.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/ProblemDefinition.h>

#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>


#include <cassert>
#include <cstdlib> // For rand()

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <ompl/base/spaces/SE3StateSpace.h>  // Include SE3StateSpace

#include "global_vars.hpp"
#include "my_motion_validator.hpp" 
#include <yaml-cpp/yaml.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Bool.h>
 #include <filesystem>
 #include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/message_event.h>
#include <ros/message.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>


extern std::vector<double> current_att_quat;
extern std::vector<double> current_vel_rate;
extern std::vector<double> current_pos_att;
extern bool new_data_received;
extern std::vector<double> current_vel_body;
extern std::vector<double> angles;
extern std::vector<double> angles_d;

extern std::vector<double> goal;
extern std::vector<double> goal_t1;



// Callback functions
void pos_cb(const nav_msgs::Odometry::ConstPtr &msg);
void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg);
void orientation_cb(const geometry_msgs::Vector3Stamped::ConstPtr &msg);
//void goal_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
void goal_cb(const geometry_msgs::PointStamped::ConstPtr& msg);

void reset_tether_cb(const std_msgs::Bool::ConstPtr& msg);
void record_trajectory_on_cb(const std_msgs::Bool::ConstPtr& msg);


bool goal_updated(const std::vector<double> &vec1, const std::vector<double> &vec2, double threshold_distance);





std::vector<std::vector<double>> sparsifyTrajectory(const std::string &filePath, double distanceThreshold) ;
double distance(const std::vector<double> &wp1, const std::vector<double> &wp2) ;

void printTrajectory(const std::vector<std::vector<double>> &trajectory) ;

void transformWaypoints(std::vector<std::vector<double>>& waypoints, 
                        float scale_factor, 
                        const Eigen::Vector3f& translation, 
                        const Eigen::Matrix3f& rotation);



//double findTetherLength(const ompl::geometric::PathGeometric &path);


bool isPointInsideCylinder(const Eigen::Vector3f& point, const cylinder_obs& cylinder);


bool isStateValid(const ompl::base::State *state);

bool isStateValid_safe(const ompl::base::State *state);

void densifyPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

void initializeVoxelGridAndKdTree(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

void transformPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float scale_factor, const Eigen::Vector3f& translation, const Eigen::Matrix3f& rotation);


//void saveTrajectoryData(const ros::Time &ros_time, 
//    const std::vector<double> &rov_pos, 
//    const std::vector<double> &angles, 
//    const ompl::geometric::PathGeometric &tether, 
//    bool record_trajectory);


void initializeTrajectoryFilename();
void recordRosbag(const std::string &bag_filename, const std::vector<std::string> &topics, ros::Duration duration);

//std_msgs::ColorRGBA loadColorFromYAML(const std::string &file_name, const std::string &color_name);



std::vector<double> crossProduct(const std::vector<double> &v1, const std::vector<double> &v2);

std::vector<double> normalize(const std::vector<double> &v);


std::vector<double> findPlaneNormal(const std::vector<double>& v1, const std::vector<double>& v2);


std::vector<double> findPerpendicularLineDirection(const std::vector<double>& v1,
                                                   const std::vector<double>& a) ;



std::vector<std::vector<double>> sampleAtDistance(const std::vector<double>& start, 
                           const std::vector<double>& direction, double delta);                                       

void saveTetherPathData(const ros::Time &ros_time, const ompl::geometric::PathGeometric &tether);


void saveTrajectory(const ros::Time &ros_time, const std::vector<double> &rov_pos);

#endif // HELPER_FUNCTIONS_HPP