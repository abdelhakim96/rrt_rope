#include "publishers.hpp"



void publishCylinders(ros::Publisher &publisher, const std::vector<cylinder_obs> &cylinders, const std::string &frame_id)
{
    visualization_msgs::MarkerArray marker_array;
    for (size_t i = 0; i < cylinders.size(); ++i)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "cylinders";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = cylinders[i].baseCenter.x();
        marker.pose.position.y = cylinders[i].baseCenter.y();
        marker.pose.position.z = cylinders[i].baseCenter.z() + cylinders[i].height / 2.0;

        // Calculate the quaternion for the cylinder's orientation
        Eigen::Vector3f z_axis(0.0, 0.0, 1.0);
        Eigen::Quaternionf quaternion = Eigen::Quaternionf::FromTwoVectors(z_axis, cylinders[i].axis);

        marker.pose.orientation.x = quaternion.x();
        marker.pose.orientation.y = quaternion.y();
        marker.pose.orientation.z = quaternion.z();
        marker.pose.orientation.w = quaternion.w();

        marker.scale.x = cylinders[i].radius * 2.0;
        marker.scale.y = cylinders[i].radius * 2.0;
        marker.scale.z = cylinders[i].height;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.4;
        marker_array.markers.push_back(marker);
    }
    publisher.publish(marker_array);
}




void publishTrajectory(const ros::Publisher &publisher, const std::vector<std::vector<double>> &trajectory) {
    // Create the message to hold the trajectory markers
    visualization_msgs::Marker marker;
    
    marker.header.frame_id = "world";  // Set the reference frame
    marker.header.stamp = ros::Time::now();
    marker.ns = "trajectory";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.w = 1.0;  // No rotation

    marker.scale.x = 0.2;  // Size of the spheres
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    marker.color.a = 0.5;  // Semi-transparent
    marker.color.r = 0.8;  // Red color
    marker.color.g = 0.4;  // Green color
    marker.color.b = 0.0;  // Blue color

    // Add the points from the trajectory as markers
    for (const auto &waypoint : trajectory) {
        if (waypoint.size() >= 3) {  // Ensure there are at least 3 elements (X, Y, Z)
            geometry_msgs::Point point;
            point.x = waypoint[0];  // X
            point.y = waypoint[1];  // Y
            point.z = waypoint[2];  // Z

            marker.points.push_back(point);
        }
    }

    // Publish the markers
    publisher.publish(marker);
    ROS_INFO("Published trajectory as markers");
}






void publishTetherPath(ros::Publisher &pub, const ompl::geometric::PathGeometric &path, const std::string &frame_id, const std_msgs::ColorRGBA &color)
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

    // Create a marker for visualization
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "tether_path";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1;  // Line width
    marker.color = color;

    for (const auto &pose : tether_path_msg.poses)
    {
        geometry_msgs::Point p;
        p.x = pose.pose.position.x;
        p.y = pose.pose.position.y;
        p.z = pose.pose.position.z;
        marker.points.push_back(p);
    }

    pub.publish(tether_path_msg);
    pub.publish(marker);
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


void publishRef(const ros::Publisher &publisher, const std::vector<double> &point) {
    if (point.size() < 4) {
        ROS_ERROR("Point must contain at least 4 elements: [x, y, z, yaw]");
        return;
    }

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id = "world";  // Set the reference frame
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.pose.position.x = point[0];  // X
    pose_msg.pose.position.y = point[1];  // Y
    pose_msg.pose.position.z = point[2];  // Z

    // Convert yaw to quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, point[3]);  // Roll and pitch are 0, yaw is point[3]
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    publisher.publish(pose_msg);
    ROS_INFO("Published pose: [%f, %f, %f, %f]", point[0], point[1], point[2], point[3]);
}
