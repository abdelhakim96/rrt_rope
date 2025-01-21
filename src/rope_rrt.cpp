#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <vector>
#include <cmath>

// Structure to represent an obstacle as a sphere
struct Obstacle
{
    std::vector<double> center; // Center of the obstacle
    double radius;              // Radius of the obstacle

    Obstacle(const std::vector<double> &c, double r) : center(c), radius(r) {}
};

// Global list of obstacles
std::vector<Obstacle> obstacles;

// Function to check if a state is valid (not in collision with any obstacle)
bool isStateValid(const ompl::base::State *state)
{
    const auto *realState = state->as<ompl::base::RealVectorStateSpace::StateType>();
    double x = realState->values[0];
    double y = realState->values[1];
    double z = realState->values[2];

    for (const auto &obstacle : obstacles)
    {
        double distSq = std::pow(x - obstacle.center[0], 2) +
                        std::pow(y - obstacle.center[1], 2) +
                        std::pow(z - obstacle.center[2], 2);
        if (distSq <= std::pow(obstacle.radius, 2))
        {
            return false; // State is in collision with this obstacle
        }
    }

    return true; // State is valid
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
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
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
void publishObstacles(ros::Publisher &obstacle_pub, const std::vector<Obstacle> &obstacles, const std::string &frame_id)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "obstacles";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.r = 1.0f;  // Red
    marker.color.b = 1.0f;  // Red
    marker.color.g = 1.0f;  // Red

    marker.color.a = 0.5;

    for (const auto &obstacle : obstacles)
    {
        geometry_msgs::Point p;
        p.x = obstacle.center[0];
        p.y = obstacle.center[1];
        p.z = obstacle.center[2];
        marker.points.push_back(p);

        // Set the scale for each obstacle based on its radius
        visualization_msgs::Marker sphere;
        sphere.header.frame_id = frame_id;
        sphere.header.stamp = ros::Time::now();
        sphere.ns = "obstacles";
        sphere.id = marker.points.size();  // Unique ID for each sphere
        sphere.type = visualization_msgs::Marker::SPHERE;
        sphere.action = visualization_msgs::Marker::ADD;
        sphere.scale.x = obstacle.radius * 2;
        sphere.scale.y = obstacle.radius * 2;
        sphere.scale.z = obstacle.radius * 2;
        sphere.color.r = 1.0f;  // Red
        sphere.color.g = 1.0f;  // Red
        sphere.color.b = 1.0f;  // Red

        sphere.color.a = 0.5;
        sphere.pose.position = p;

        obstacle_pub.publish(sphere);
    }

    obstacle_pub.publish(marker);
}

int main(int argc, char **argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "rope_shortcut_path_example");
    ros::NodeHandle nh;

    // Create publishers to visualize the original and optimized paths, and obstacles
    ros::Publisher rov_path_pub = nh.advertise<visualization_msgs::Marker>("rov_path", 10);
    ros::Publisher rope_path_pub = nh.advertise<visualization_msgs::Marker>("rope_path", 10);
    ros::Publisher obstacle_pub = nh.advertise<visualization_msgs::Marker>("obstacles", 10);

    // Define obstacles (center and radius)
    obstacles.emplace_back(std::vector<double>{0.0, 0.0, 0.0}, 2.0);  // Obstacle at (0, 2.5, 0) with radius 1

    ros::Rate rate(2);

    while (ros::ok())
    {
        // Define a 3D state space
        auto space = std::make_shared<ompl::base::RealVectorStateSpace>(3);
        ompl::base::RealVectorBounds bounds(3);
        bounds.setLow(-10);
        bounds.setHigh(15);
        space->setBounds(bounds);

        // Create space information and set the state validity checker
        auto si = std::make_shared<ompl::base::SpaceInformation>(space);
        si->setStateValidityChecker(isStateValid);

        // Define a geometric path in the space (Semi-Circular Path)
        ompl::geometric::PathGeometric path(si);
        int num_points = 50;  // Number of points for the semi-circle
        double radius = 5.0;
        for (int i = 0; i <= num_points; ++i)
        {
            double angle = M_PI * i / num_points;  // Semi-circle from 0 to π
            auto *state = si->allocState()->as<ompl::base::RealVectorStateSpace::StateType>();
            state->values[0] = radius * cos(angle);  // x-coordinate
            state->values[1] = radius * sin(angle);  // y-coordinate
            state->values[2] = 0;                    // z-coordinate (keeping it flat)
            path.append(state);
        }

        // Publish the original path
        ROS_INFO("Publishing original path...");
        std_msgs::ColorRGBA rovpathColor;
        rovpathColor.r = 0.0f;  // Red
        rovpathColor.g = 0.0f;  // Green
        rovpathColor.b = 1.0f;  // Blue
        rovpathColor.a = 1.0f;  // Alpha (transparency)
        publishPath(rov_path_pub, path, "world", "rov_path", rovpathColor);

        // Simplify the path using ropeShortcutPath (Optimized Path)
        ompl::geometric::PathSimplifier simplifier(si);
        double delta = 1;                // Step size
        double equivalenceTolerance = 0.5;  // Equivalence tolerance
        ROS_INFO("Applying ropeShortcutPath...");
        bool improved = simplifier.ropeShortcutPath(path, delta, equivalenceTolerance);

        // Publish the optimized path
        ROS_INFO("Publishing optimized path...");
        std_msgs::ColorRGBA ropepathColor;
        ropepathColor.r = 0.0f;  // Red
        ropepathColor.g = 1.0f;  // Green
        ropepathColor.b = 0.0f;  // Blue
        ropepathColor.a = 1.0f;  // Alpha (transparency)
        publishPath(rope_path_pub, path, "world", "rope_path", ropepathColor);

        if (improved)
        {
            ROS_INFO("Path improved with ropeShortcutPath.");
        }
        else
        {
            ROS_INFO("No improvements made to the path.");
        }

        // Publish the obstacles
        publishObstacles(obstacle_pub, obstacles, "world");

        // Keep the node spinning to visualize continuously
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}