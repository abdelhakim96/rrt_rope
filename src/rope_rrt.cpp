
#include <rope_rrt.hpp>
// Structure to represent an obstacle as a sphere

std::vector<double> current_att_quat(4, 0.0);  // Quaternion has 4 components
std::vector<double> current_vel_rate(3, 0.0); // Velocity rate has 3 components
std::vector<double> current_pos_att(3, 0.0);  // Position and attitude have 3 components
bool new_data_received = false;
std::vector<double> current_vel_body(3, 0.0); // Velocity body has 3 components
std::vector<double> angles(3, 0.0);           // Angles have 3 components
std::vector<double> angles_d(3, 0.0);         // Angles in degrees have 3 components




void pos_cb(const nav_msgs::Odometry::ConstPtr &msg) {
    current_att_quat = {
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w};
    current_vel_rate = {
        msg->twist.twist.linear.x,
        msg->twist.twist.linear.y,
        msg->twist.twist.linear.z,
        msg->twist.twist.angular.x,
        msg->twist.twist.angular.y,
        msg->twist.twist.angular.z};
    current_pos_att = {
        msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z, 0.0, 0.0, 0.0}; // roll, pitch, yaw can be computed
    new_data_received = true;
}

void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg) {
    current_vel_body = {
        msg->twist.linear.x,
        msg->twist.linear.y,
        msg->twist.linear.z,
        msg->twist.angular.x,
        msg->twist.angular.y,
        msg->twist.angular.z};
}

void orientation_cb(const geometry_msgs::Vector3Stamped::ConstPtr &msg) {
    angles = {msg->vector.x * (M_PI / 180),
              msg->vector.y * (M_PI / 180),
              msg->vector.z * (M_PI / 180)};
    angles_d = {msg->vector.x, msg->vector.y, msg->vector.z};
}




















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

// Custom motion validator class
class myMotionValidator : public ompl::base::MotionValidator
{
public:
    myMotionValidator(const ompl::base::SpaceInformationPtr &si) : ompl::base::MotionValidator(si) {}

    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const override
    {
        // Check if the start and end states are valid
        if (!si_->isValid(s1) || !si_->isValid(s2))
            return false;

        // Interpolate between the states and check each intermediate state for validity
        int nd = si_->getStateSpace()->validSegmentCount(s1, s2);
        auto *interpolatedState = si_->allocState();

        for (int i = 1; i < nd; ++i)
        {
            si_->getStateSpace()->interpolate(s1, s2, (double)i / (double)nd, interpolatedState);
            if (!si_->isValid(interpolatedState))
            {
                si_->freeState(interpolatedState);
                return false;
            }
        }

        si_->freeState(interpolatedState);
        return true;
    }

    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2, std::pair<ompl::base::State *, double> &lastValid) const override
    {
        // Check if the start state is valid
        if (!si_->isValid(s1))
        {
            lastValid.first = si_->cloneState(s1);
            lastValid.second = 0.0;
            return false;
        }

        // Interpolate between the states and check each intermediate state for validity
        int nd = si_->getStateSpace()->validSegmentCount(s1, s2);
        auto *interpolatedState = si_->allocState();

        for (int i = 1; i < nd; ++i)
        {
            si_->getStateSpace()->interpolate(s1, s2, (double)i / (double)nd, interpolatedState);
            if (!si_->isValid(interpolatedState))
            {
                lastValid.first = si_->cloneState(interpolatedState);
                lastValid.second = (double)(i - 1) / (double)nd;
                si_->freeState(interpolatedState);
                return false;
            }
        }

        si_->freeState(interpolatedState);
        lastValid.second = 1.0;
        return true;
    }
};





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
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
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
    ros::Time ros_time; 
    int count = 0;
    // Create publishers to visualize the original and optimized paths, and obstacles
    ros::Publisher rov_path_pub = nh.advertise<visualization_msgs::Marker>("rov_path", 10);
    ros::Publisher rope_path_pub = nh.advertise<visualization_msgs::Marker>("rope_path", 10);
    ros::Publisher obstacle_pub = nh.advertise<visualization_msgs::Marker>("obstacle", 10);
    ros::Publisher tether_path_pub = nh.advertise<nav_msgs::Path>("rope_rrt_tether_path", 10);


    //subscribers
    ros::Subscriber pos_sub = nh.subscribe<nav_msgs::Odometry>("/mobula/rov/odometry", 1, pos_cb);
    ros::Subscriber orientation_sub = nh.subscribe<geometry_msgs::Vector3Stamped>("/mobula/rov/orientation", 1, orientation_cb);

    // Define obstacles (center and radius)
    ros_time = ros::Time::now();



    ros::Rate rate(1);
   auto space = std::make_shared<ompl::base::RealVectorStateSpace>(3);
        ompl::base::RealVectorBounds bounds(3);
        bounds.setLow(-10);
        bounds.setHigh(15);
        space->setBounds(bounds);
        
       // Create space information and set the state validity checker
        auto si = std::make_shared<ompl::base::SpaceInformation>(space);
        si->setStateValidityChecker(isStateValid);

        // Set the custom motion validator
        si->setMotionValidator(std::make_shared<myMotionValidator>(si));
        si->setup();

        // Define a geometric path in the space (Semi-Circular Path)
        ompl::geometric::PathGeometric path1(si);
        ompl::geometric::PathGeometric path2(si);

        std::vector<ompl::base::State *> contactPoints;

    while (ros::ok())
    {

        double o_radius = 2 * cos(0.2  * count);
        count++;
        //ROS_ERROR("TIME: %d", count);
        //ROS_ERROR("Radius: %f", o_radius);
        // Ensure the obstacles vector has at least one element
        if (obstacles.empty()) {
            obstacles.emplace_back(std::vector<double>{0.0, 0.0, 0.0}, 1.0);  // Initial obstacle
            
        }
        double pos_x = -1.0;
        double pos_y = 2 * cos(0.2  * count);
        double pos_z = 1 * sin(0.2  * count);
        o_radius = 0.5;
        pos_y = -0.0;
        pos_z = 0.0; 
        // Update the first obstacle
        obstacles[0] = Obstacle(std::vector<double>{pos_x, pos_y, pos_z+0.1}, o_radius);  // Update the first obstacle


       obstacles.emplace_back(std::vector<double>{pos_x, pos_y+ 2.0, pos_z}, o_radius/2);  // Initial obstacle


        // Define a 3D state space
        auto space = std::make_shared<ompl::base::RealVectorStateSpace>(3);
        ompl::base::RealVectorBounds bounds(3);
        bounds.setLow(-10);
        bounds.setHigh(15);
        space->setBounds(bounds);
        
 
        int num_points = 10;  // Number of points for the semi-circle
        double radius = 3.0;
        
        //current_pos_att
        
        double rotation_angle = 20.0 * cos(0.2 * 0.0);  // Rotation amount
     
     //if (count<2){
     //   for (int i = 0; i <= num_points; ++i)
     //   {
      //      double angle = 3 * M_PI_2 * i / num_points;  // Three-quarters of a circle from 0 to 3π/2

            // Rotation about z-axis
      //      auto *state = si->allocState()->as<ompl::base::RealVectorStateSpace::StateType>();
      //      state->values[0] = radius * cos(angle) * cos(rotation_angle) - radius * sin(angle) * sin(rotation_angle);  // x-coordinate
      //      state->values[1] = radius * cos(angle) * sin(rotation_angle) + radius * sin(angle) * cos(rotation_angle);  // y-coordinate
      //      state->values[2] = 0;  // z-coordinate (keeping it flat)
      //      path1.append(state);
       // }
    // }
        
        //path.clear();  // Clear the existing path
       si->setStateValidityChecker(isStateValid);

        // Set the custom motion validator
        si->setMotionValidator(std::make_shared<myMotionValidator>(si));
        si->setup();

        // Define a geometric path in the space (Semi-Circular Path)
       // path(si);

        auto *state = si->allocState()->as<ompl::base::RealVectorStateSpace::StateType>();
       state->values[0] = current_pos_att[0];  // x-coordinate
       state->values[1] = current_pos_att[1];  // y-coordinate
        state->values[2] = current_pos_att[2];  // z-coordinate (keeping it flat)
        path1.append(state);

        // Publish the original path
        //ROS_INFO("Publishing original path...");
        std_msgs::ColorRGBA rovpathColor;
        rovpathColor.r = 0.0f;  // Red
        rovpathColor.g = 0.0f;  // Green
        rovpathColor.b = 1.0f;  // Blue
        rovpathColor.a = 1.0f;  // Alpha (transparency)
        publishPath(rov_path_pub, path1, "world", "rov_path", rovpathColor);

        // Simplify the path using ropeShortcutPath (Optimized Path)
        ompl::geometric::PathSimplifier simplifier(si);
        double delta = 0.5;                // Step size
        double equivalenceTolerance = 0.01;  // Equivalence tolerance
        //ROS_INFO("Applying ropeShortcutPath...");
        bool improved;
        //if (count<2){
        //path2 = path1;
        improved = simplifier.ropeRRTtether(path1, contactPoints, delta, equivalenceTolerance);


/*
if (contactPoints.empty())
{
    std::cout << "  No contact points found." << std::endl;
}
else
{
    std::cout << "  Number of contact points: " << contactPoints.size()-1 << std::endl;
    for (size_t idx = 0; idx < contactPoints.size()-1; ++idx)
    {
        const auto *contact = contactPoints[idx];
        if (contact == nullptr)
        {
            std::cout << "  Invalid contact point (nullptr) at index " << idx << std::endl;
            continue;
        }

        const auto *state = contact->as<ompl::base::RealVectorStateSpace::StateType>();
        if (state == nullptr)
        {
            std::cout << "  Invalid state (nullptr) at index " << idx << std::endl;
            continue;
        }

        std::cout << "  Contact Point " << idx << ": (" << state->values[0] << ", " << state->values[1] << ", " << state->values[2] << ")" << std::endl;
    }
}

*/
        // Check if the optimized path is valid
        bool pathValid = true;
        for (size_t i = 0; i < path1.getStateCount(); ++i)
        {
            if (!isStateValid(path1.getState(i)))
            {
                pathValid = false;
                ROS_WARN("Optimized path goes through an obstacle at state %zu", i);
                break;
            }
        }

        // Publish the optimized path
        ROS_INFO("Publishing optimized path...");
        std_msgs::ColorRGBA ropepathColor;
        ropepathColor.r = 0.0f;  // Red
        ropepathColor.g = 1.0f;  // Green
        ropepathColor.b = 0.0f;  // Blue
        ropepathColor.a = 1.0f;  // Alpha (transparency)
        publishPath(rope_path_pub, path1, "world", "rope_path", ropepathColor);
         

        publishTetherPath(tether_path_pub, path1, "world");


        if (improved && pathValid)
        {
            ROS_INFO("Path improved with ropeShortcutPath and is valid.");
        }
        else if (improved)
        {
            ROS_WARN("Path improved with ropeShortcutPath but is not valid.");
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