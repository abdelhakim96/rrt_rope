#include "helper_functions.hpp"


std::vector<double> current_att_quat(4, 0.0);  // Quaternion has 4 components
std::vector<double> current_vel_rate(3, 0.0); // Velocity rate has 3 components
std::vector<double> current_pos_att(3, 0.0);  // Position and attitude have 3 components
bool new_data_received = false;
std::vector<double> current_vel_body(3, 0.0); // Velocity body has 3 components
std::vector<double> angles(3, 0.0);           // Angles have 3 components
std::vector<double> angles_d(3, 0.0);         // Angles in degrees have 3 components

 std::vector<double> goal(3, 0.0); 
std::vector<double> goal_t1(3, 0.0); 






bool isStateValid(const ompl::base::State *state)
{
    // Cast the state to RealVectorStateSpace::StateType
    const auto *realState = state->as<ompl::base::RealVectorStateSpace::StateType>();
    if (!realState)
    {
        // ROS_ERROR("State is not of type RealVectorStateSpace::StateType.");
        return false;
    }

    // Access the position values
    double x = realState->values[0];
    double y = realState->values[1];
    double z = realState->values[2];

    // Convert position to Eigen vector
    Eigen::Vector3f robot_position(x, y, z);

    // Print the robot's position
    // ROS_INFO("Checking state at position: [%f, %f, %f]", x, y, z);

    // Check if the robot's position is inside any cylinder
    for (const auto& cylinder : cylinders)
    {
        if (isPointInsideCylinder(robot_position, cylinder))
        {
           // ROS_WARN("Collision detected with cylinder.");
            return false; // Collision detected with cylinder
        }
    }

    // ROS_INFO("State is valid.");
    return true; // No collision
}

















bool isPointInsideCylinder(const Eigen::Vector3f& point, const cylinder_obs& cylinder)
{
    // Print the base center of the cylinder
    //std::cout << "Cylinder Base Center: " << cylinder.baseCenter.transpose() << std::endl;

    // Compute the vector from the base center to the point
    Eigen::Vector3f baseToPoint;
    baseToPoint.x() = point.x() - cylinder.baseCenter.x();
    baseToPoint.y() = point.y() - cylinder.baseCenter.y();
    baseToPoint.z() = point.z() + cylinder.baseCenter.z();

    // Print the baseToPoint
    //std::cout << "Base to Point: " << baseToPoint.transpose() << std::endl;

    // Normalize the axis of the cylinder
    Eigen::Vector3f axis = cylinder.axis.normalized();

    // Project the vector onto the cylinder's axis to get the height component
    float projectionLength;
    if (axis == Eigen::Vector3f(0, 0, 1))
    {
        // If the cylinder extends along the z-axis, use the z-component for projection length
        projectionLength = baseToPoint.z();
    }
    else
    {
        // Otherwise, project the vector onto the cylinder's axis
        projectionLength = -baseToPoint.x();
    }

    // Print the projection length
    //std::cout << "Projection Length: " << projectionLength << std::endl;

    // Check if the projection is within the cylinder's height range
    if (projectionLength < 0 || projectionLength > cylinder.height)
    {
        return false;
    }

    // Compute the closest point on the cylinder axis
    Eigen::Vector3f closestPointOnAxis = cylinder.baseCenter + projectionLength * axis;

    // Print the closest point on the axis
    //std::cout << "Closest Point on Axis: " << closestPointOnAxis.transpose() << std::endl;

    // Compute the radial distance based on the cylinder's axis
    float radialDistance;
    if (axis == Eigen::Vector3f(0, 0, 1))
    {
        // If the cylinder extends along the z-axis, compute the radial distance in the XY plane
        radialDistance = std::sqrt(baseToPoint.x() * baseToPoint.x() + baseToPoint.y() * baseToPoint.y());
    }
    else
    {
        // Compute the radial distance by removing the axial component
         radialDistance = std::sqrt(baseToPoint.z() * baseToPoint.z() + baseToPoint.y() * baseToPoint.y());
    }

    // Print the radial distance
    //std::cout << "Radial Distance: " << radialDistance << std::endl;

    // Check if the point is within the cylinder's radius
    bool isInside = radialDistance <= cylinder.radius;

    // Print the result
    //std::cout << "Is Inside: " << isInside << std::endl;

    return isInside;
}








 
// Custom motion validator class


bool goal_updated(const std::vector<double> &vec1, const std::vector<double> &vec2, double threshold_distance) {
    if (vec1.size() != vec2.size()) {
        return true;
    }
    double distance_squared = 0.0;
    for (size_t i = 0; i < vec1.size(); ++i) {
        distance_squared += (vec1[i] - vec2[i]) * (vec1[i] - vec2[i]);
    }
    return distance_squared > (threshold_distance * threshold_distance);
}



void reset_tether_cb(const std_msgs::Bool::ConstPtr& msg) {
    reset_tether = msg->data;  // Assign the data from the message to the boolean

}

void record_trajectory_on_cb(const std_msgs::Bool::ConstPtr& msg) {
    record_trajectory = msg->data;  // Assign the data from the message to the boolean
}



void goal_cb(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    // Process the received goal point
    //ROS_INFO("Received goal point: [x: %f, y: %f, z: %f]", msg->point.x, msg->point.y, msg->point.z);
    // Update the goal with the received point
    goal[0] = msg->point.x;
    goal[1] = msg->point.y;
    goal[2] = msg->point.z;
}



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




////////////////////////
////////////////////////




// Function to compute the Euclidean distance between two waypoints
double distance(const std::vector<double> &wp1, const std::vector<double> &wp2) {
    return std::sqrt(std::pow(wp2[0] - wp1[0], 2) + // X
                     std::pow(wp2[1] - wp1[1], 2) + // Y
                     std::pow(wp2[2] - wp1[2], 2));  // Z
}

std::vector<std::vector<double>> sparsifyTrajectory(const std::string &filePath, double distanceThreshold) {
    std::vector<std::vector<double>> way_point_traj;
    std::ifstream inputFile(filePath);
    
    if (!inputFile.is_open()) {
        std::cerr << "Error opening file!" << std::endl;
        return way_point_traj;
    }
    
    std::string line;
    std::vector<double> previousWaypoint = {0, 0, 0, 0};  // Initialize previous waypoint
    bool isFirstPoint = true;
    
    while (std::getline(inputFile, line)) {
        std::stringstream ss(line);
        std::string timestampStr, xStr, yStr, zStr, pitchStr, yawStr;
        
        std::getline(ss, timestampStr, ',');
        std::getline(ss, xStr, ',');
        std::getline(ss, yStr, ',');
        std::getline(ss, zStr, ',');
        std::getline(ss, pitchStr, ',');
        std::getline(ss, yawStr, ',');
        
        double x = std::stod(xStr.substr(3)); // Remove 'X: ' part
        double y = std::stod(yStr.substr(3)); // Remove 'Y: ' part
        double z = std::stod(zStr.substr(3)); // Remove 'Z: ' part
        double yaw = std::stod(yawStr.substr(5)); // Remove 'YAW: ' part
        
        // Sparsify based on the Euclidean distance threshold
        std::vector<double> currentWaypoint = {x, y, z, yaw};
        if (isFirstPoint || distance(previousWaypoint, currentWaypoint) > distanceThreshold) {
            way_point_traj.push_back(currentWaypoint);
            previousWaypoint = currentWaypoint;
            isFirstPoint = false;
        }
    }
    
    inputFile.close();
    
    return way_point_traj;
}




void transformWaypoints(std::vector<std::vector<double>>& waypoints, 
                        float scale_factor, 
                        const Eigen::Vector3f& translation, 
                        const Eigen::Matrix3f& rotation)
{
    ROS_INFO("Transforming waypoints.");

    for (auto& waypoint : waypoints)
    {
        // Convert waypoint to Eigen::Vector3f for transformation (assuming waypoint = {x, y, z, yaw})
        Eigen::Vector3f p(waypoint[0] * scale_factor, waypoint[1] * scale_factor, waypoint[2] * scale_factor);

        // Apply rotation
        p = rotation * p;

        // Apply translation
        waypoint[0] = p.x() + translation.x();  // Transformed X
        waypoint[1] = p.y() + translation.y();  // Transformed Y
        waypoint[2] = p.z() + translation.z();  // Transformed Z
    }
}








void printTrajectory(const std::vector<std::vector<double>> &trajectory) {
    for (const auto &wp : trajectory) {
        std::cout << "X: " << wp[0] << ", Y: " << wp[1] << ", Z: " << wp[2] << ", YAW: " << wp[3] << std::endl;
    }
}



/////////////

// Function to find the length of the tether path
/*
double findTetherLength(const ompl::geometric::PathGeometric &path)
{
    double length = 0.0;

    // Iterate through the states in the path
    for (std::size_t i = 1; i < path.getStateCount(); ++i)
    {
        // Get the current and previous states
        const auto *state1 = path.getState(i - 1)->as<ompl::base::RealVectorStateSpace::StateType>();
        const auto *state2 = path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();

        // Convert the states to Eigen vectors
        Eigen::Vector3d p1(state1->values[0], state1->values[1], state1->values[2]);
        Eigen::Vector3d p2(state2->values[0], state2->values[1], state2->values[2]);

        // Calculate the distance between the states and add to the total length
        length += (p2 - p1).norm();
    }

    return length;
}
*/
//Eigen::vector3d findExitPoint(const ompl::geometric::PathGeometric &path, double radius)










void densifyPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    // Check if the input cloud is empty
    if (cloud->empty())
    {
        PCL_ERROR("Input cloud is empty!\n");
        return;
    }
 
    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
 
    // Output point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr mls_points(new pcl::PointCloud<pcl::PointXYZ>);
 
    // Initialize object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
    mls.setComputeNormals(false);
 
    // Set parameters
    mls.setInputCloud(cloud);
    mls.setPolynomialFit(true);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.03); // Adjust the search radius as needed
 
    // Reconstruct
    mls.process(*mls_points);
 
    // Check if the output cloud is empty
    if (mls_points->empty())
    {
        PCL_ERROR("Densified cloud is empty!\n");
        return;
    }
 
    // Update the input cloud with the densified points
    cloud = mls_points;
}
 
 
 
 
 
 
void initializeVoxelGridAndKdTree(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    //ROS_INFO("Initializing voxel grid and k-d tree.");
 
    // Downsample the point cloud using a voxel grid filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(0.01f, 0.01f, 0.01f); // Adjust the leaf size for higher resolution
    voxel_grid.filter(*filtered_cloud);
 
    //ROS_INFO("Voxel grid filter applied. Original points: %zu, Filtered points: %zu", cloud->points.size(), filtered_cloud->points.size());
 
    // Initialize the k-d tree with the downsampled point cloud
    kdtree.setInputCloud(filtered_cloud);
    voxel_grid_initialized = true;
 
    //ROS_INFO("K-d tree initialized with filtered point cloud.");
}
 
 

 
void transformPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float scale_factor, const Eigen::Vector3f& translation, const Eigen::Matrix3f& rotation)
{  
    ROS_INFO("Transforming point cloud.");
    for (auto& point : cloud->points)
    {
        // Apply scaling
        Eigen::Vector3f p(point.x * scale_factor, point.y * scale_factor, point.z * scale_factor);
 
        // Apply rotation
        p = rotation * p;
 
        // Apply translation
        point.x = p.x() + translation.x();
        point.y = p.y() + translation.y();
        point.z = p.z() + translation.z();
    }
}



//Data Collection Functions
void initializeTrajectoryFilename()
{
    // Directory to save the files
    std::string directory = "/home/hakim/tether_planning_ws/src/rope_rrt/results/";

    // Ensure the directory exists
    std::filesystem::create_directories(directory);

    // Find the highest numbered file in the directory
    int max_number = 0;
    for (const auto &entry : std::filesystem::directory_iterator(directory))
    {
        std::string filename = entry.path().filename().string();
        if (filename.find("trajectory_results_") == 0 && filename.find(".txt") != std::string::npos)
        {
            int number = std::stoi(filename.substr(18, filename.size() - 22));
            if (number > max_number)
            {
                max_number = number;
            }
        }
    }

    // Generate a new filename with an incremented number
    std::ostringstream filename;
    filename << directory << "trajectory_results_" << (max_number + 1) << ".txt";
    trajectory_filename = filename.str();
}

void saveTrajectoryData(const ros::Time &ros_time, 
                      const std::vector<double> &rov_pos, 
                      const std::vector<double> &angles, 
                      const ompl::geometric::PathGeometric &tether, 
                      bool record_trajectory)
{
    if (!record_trajectory)
    {
        return;
    }

    // Open the file for writing
    std::ofstream file(trajectory_filename);
    if (!file.is_open())
    {
        ROS_ERROR("Failed to open file: %s", trajectory_filename.c_str());
        return;
    }

    // Write the header
    file << "ros_time, rov_pos_x, rov_pos_y, rov_pos_z, yaw, pitch, roll, tether_x, tether_y, tether_z\n";

    // Write the data row by row
    for (std::size_t i = 0; i < tether.getStateCount(); ++i)
    {
        const auto *state = tether.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
        file << std::fixed << std::setprecision(6)
             << ros_time.toSec() << ", "
             << rov_pos[0] << ", " << rov_pos[1] << ", " << rov_pos[2] << ", "
             << angles[0] << ", " << angles[1] << ", " << angles[2] << ", "
             << state->values[0] << ", " << state->values[1] << ", " << state->values[2] << "\n";
    }

    // Close the file
    file.close();
    ROS_INFO("Trajectory data saved to file: %s", trajectory_filename.c_str());
}



/*
void recordRosbag(const std::string &bag_filename, const std::vector<std::string> &topics, ros::Duration duration)
{
    rosbag::Bag bag;
    bag.open(bag_filename, rosbag::bagmode::Write);

    ros::NodeHandle nh;
    std::vector<ros::Subscriber> subscribers;

    auto callback = [&bag](const ros::MessageEvent<ros::Message const> &event) {
        const std::string &topic = event.getConnectionHeader()["topic"];
        const ros::Time &time = event.getReceiptTime();
        const boost::shared_ptr<ros::Message const> &msg = event.getMessage();
        bag.write(topic, time, msg);
    };

    for (const auto &topic : topics)
    {
        subscribers.push_back(nh.subscribe(topic, 1000, callback));
    }

    ros::Time start_time = ros::Time::now();
    while (ros::ok() && (ros::Time::now() - start_time) < duration)
    {
        ros::spinOnce();
    }

    bag.close();
    ROS_INFO("Rosbag recording saved to %s", bag_filename.c_str());
}

*/
