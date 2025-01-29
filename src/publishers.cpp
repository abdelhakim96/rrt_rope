#include "publishers.hpp"

void publishTetherPath(ros::Publisher &pub, const ompl::geometric::PathGeometric &path, const std::string &frame_id)
{
    nav_msgs::Path tether_path_msg;
    tether_path_msg.header.frame_id = frame_id;
    tether_path_msg.header.stamp = ros::Time::now();

    for (size_t i = 0; i < path.getStateCount(); ++i)
    {
        const auto *state = path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = frame_id;
        pose.header.stamp = ros::Time::now();  // Use current time
        pose.pose.position.x = state->values[0];
        pose.pose.position.y = state->values[1];
        pose.pose.position.z = state->values[2];
        pose.pose.orientation.w = 1.0;  // Default orientation (can be updated for full 6DOF path if needed)
        tether_path_msg.poses.push_back(pose);
    }

    pub.publish(tether_path_msg);
}






void publishPointCloud(ros::Publisher &pub, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = "world";
    output.header.stamp = ros::Time::now();
    pub.publish(output);
}




void publishVoxelGrid(ros::Publisher &pub, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = "world";
    output.header.stamp = ros::Time::now();
    pub.publish(output);
}








// Function to create and publish a path
void publishPath(ros::Publisher &pub, const ompl::geometric::PathGeometric &path, const std::string &frame_id, const std::string &ns, const std_msgs::ColorRGBA &color)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.color = color;  // Use the provided color

    for (size_t i = 0; i < path.getStateCount(); ++i)
    {
        const auto *state = path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
        geometry_msgs::Point p;
        p.x = state->values[0];
        p.y = state->values[1];
        p.z = state->values[2];
        marker.points.push_back(p);
    }

    pub.publish(marker);
}

// Function to create and publish obstacles
