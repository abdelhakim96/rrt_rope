
#include <rope_rrt.hpp>

// Structure to represent an obstacle as a sphere

std::vector<double> current_att_quat(4, 0.0);  // Quaternion has 4 components
std::vector<double> current_vel_rate(3, 0.0); // Velocity rate has 3 components
std::vector<double> current_pos_att(3, 0.0);  // Position and attitude have 3 components
bool new_data_received = false;
std::vector<double> current_vel_body(3, 0.0); // Velocity body has 3 components
std::vector<double> angles(3, 0.0);           // Angles have 3 components
std::vector<double> angles_d(3, 0.0);         // Angles in degrees have 3 components

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

bool voxel_grid_initialized = false;
pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;


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





void initializeVoxelGridAndKdTree(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    ROS_INFO("Initializing voxel grid and k-d tree.");

    // Downsample the point cloud using a voxel grid filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(0.05f, 0.05f, 0.05f); // Adjust the leaf size for higher resolution
    voxel_grid.filter(*filtered_cloud);

    ROS_INFO("Voxel grid filter applied. Original points: %zu, Filtered points: %zu", cloud->points.size(), filtered_cloud->points.size());

    // Initialize the k-d tree with the downsampled point cloud
    kdtree.setInputCloud(filtered_cloud);
    voxel_grid_initialized = true;

    ROS_INFO("K-d tree initialized with filtered point cloud.");
}














struct Obstacle
{
    std::vector<double> center; // Center of the obstacle
    double radius;              // Radius of the obstacle

    Obstacle(const std::vector<double> &c, double r) : center(c), radius(r) {}
};

// Global list of obstacles
std::vector<Obstacle> obstacles;



bool isStateValid(const ompl::base::State *state)
{
    if (!voxel_grid_initialized)
    {
        ROS_ERROR("Voxel grid not initialized.");
        return false;
    }

    // Cast the state to RealVectorStateSpace::StateType
    const auto *realState = state->as<ompl::base::RealVectorStateSpace::StateType>();
    if (!realState)
    {
        ROS_ERROR("State is not of type RealVectorStateSpace::StateType.");
        return false;
    }

    // Access the position values
    double x = realState->values[0];
    double y = realState->values[1];
    double z = realState->values[2];

    // Convert position to Eigen vector
    Eigen::Vector3f robot_position(x, y, z);

    // Print the robot's position
    ROS_INFO("Checking state at position: [%f, %f, %f]", x, y, z);

    // Define a collision threshold delta
    float collision_threshold = 0.01; // Adjust as needed

    // Check if the robot's position falls within an occupied voxel using k-d tree
    pcl::PointXYZ search_point(robot_position[0], robot_position[1], robot_position[2]);
    std::vector<int> indices(1);
    std::vector<float> distances(1);
    if (kdtree.nearestKSearch(search_point, 1, indices, distances) > 0 && distances[0] < collision_threshold)
    {
        ROS_WARN("Collision detected with point at distance %f", distances[0]);
        return false; // Collision detected
    }

    ROS_INFO("State is valid.");
    return true; // No collision
}



// Custom motion validator class
// Custom motion validator class
class myMotionValidator : public ompl::base::MotionValidator
{
public:
    myMotionValidator(const ompl::base::SpaceInformationPtr &si) : ompl::base::MotionValidator(si) {}

    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const override
    {
        //ROS_INFO("Checking motion between states.");

        // Check if the start and end states are valid
       // if (!si_->isValid(s1) || !si_->isValid(s2))
        //{
       //     ROS_WARN("Start or end state is invalid.");
       //     return false;
      //  }

        // Interpolate between the states and check each intermediate state for validity
        int nd = si_->getStateSpace()->validSegmentCount(s1, s2);
        auto *interpolatedState = si_->allocState();

        for (int i = 1; i < nd; ++i)
        {
            si_->getStateSpace()->interpolate(s1, s2, (double)i / (double)nd, interpolatedState);
            if (!si_->isValid(interpolatedState))
            {
                ROS_WARN("Intermediate state %d is invalid.", i);
                si_->freeState(interpolatedState);
                return false;
            }
        }

        si_->freeState(interpolatedState);
        //ROS_INFO("Motion is valid.");
        return true;
    }

    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2, std::pair<ompl::base::State *, double> &lastValid) const override
    {
        //ROS_INFO("Checking motion between states with last valid state.");

        // Check if the start state is valid
        if (!si_->isValid(s1))
        {
            lastValid.first = si_->cloneState(s1);
            lastValid.second = 0.0;
            ROS_WARN("Start state is invalid.");
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
                ROS_WARN("Intermediate state %d is invalid.", i);
                return false;
            }
        }

        si_->freeState(interpolatedState);
        lastValid.second = 1.0;
        ROS_INFO("Motion is valid.");
        return true;
    }
};



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
    ros::Publisher point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("point_cloud", 1);   ;
    ros::Publisher voxel_grid_pub = nh.advertise<sensor_msgs::PointCloud2>("voxel_grid", 1);

    //subscribers
    ros::Subscriber pos_sub = nh.subscribe<nav_msgs::Odometry>("/mobula/rov/odometry", 1, pos_cb);
    ros::Subscriber orientation_sub = nh.subscribe<geometry_msgs::Vector3Stamped>("/mobula/rov/orientation", 1, orientation_cb);

    // Define obstacles (center and radius)
    ros_time = ros::Time::now();
    ros::Rate rate(10);

   //pcl
     float scale_factor = 0.1; // Scale down by 10 times (0.2 * 0.5)
        Eigen::Vector3f translation(2.0, 1.0, 1.0); // Move closer to the origin
        Eigen::Matrix3f rotation;
        rotation = Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitX());
        std::string filename = "/home/hakim/tether_planning_ws/src/rope_rrt/pipe.pcd";
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1)
            {
                PCL_ERROR("Couldn't read file %s \n", filename.c_str());
                return -1;
            }

        transformPointCloud(cloud, scale_factor, translation, rotation);

       //voxel grid

        initializeVoxelGridAndKdTree(cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);   
        //transformPointCloud(filtered_cloud, scale_factor, translation, rotation);
        voxel_grid.filter(*filtered_cloud);


       //create space and path


        auto space = std::make_shared<ompl::base::RealVectorStateSpace>(3);
        ompl::base::RealVectorBounds bounds(3);
        bounds.setLow(-10);
        bounds.setHigh(10);
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
        
        //define contact points
        std::vector<ompl::base::State *> contactPoints;



    while (ros::ok())
    {
            initializeVoxelGridAndKdTree(cloud);

        //auto *state = si->allocState()->as<ompl::base::RealVectorStateSpace::StateType>();
        //if (!state)
        //{
        //    ROS_ERROR("Failed to allocate state.");
        //    continue;
       // }
         //    si->setStateValidityChecker(isStateValid);

        // Set the custom motion validator
        //si->setMotionValidator(std::make_shared<myMotionValidator>(si));
       //si->setup();
        
         auto space = std::make_shared<ompl::base::RealVectorStateSpace>(3);
        ompl::base::RealVectorBounds bounds(3);
        bounds.setLow(-10);
        bounds.setHigh(10);
        space->setBounds(bounds);


        si->setStateValidityChecker(isStateValid);

        // Set the custom motion validator
         si->setMotionValidator(std::make_shared<myMotionValidator>(si));
         si->setup();
        auto *state = si->allocState()->as<ompl::base::RealVectorStateSpace::StateType>();

        state->values[0] = current_pos_att[0];  // x-coordinate
        state->values[1] = current_pos_att[1];  // y-coordinate
         state->values[2] = current_pos_att[2];  // z-coordinate (keeping it flat)
        path1.append(state);

        // Publish the original path
        std_msgs::ColorRGBA rovpathColor;
        rovpathColor.r = 0.0f;  // Red
        rovpathColor.g = 0.0f;  // Green
        rovpathColor.b = 1.0f;  // Blue
        rovpathColor.a = 1.0f;  // Alpha (transparency)

        // Simplify the path using ropeShortcutPath (Optimized Path)
        ompl::geometric::PathSimplifier simplifier(si);
        double delta = 1.0;                // Step size
        double equivalenceTolerance = 0.001;  // Equivalence tolerance
       
        // Measure the time taken by the ropeRRTtether method
        auto start_time = std::chrono::high_resolution_clock::now();
        bool improved = simplifier.ropeRRTtether(path1, contactPoints, delta, equivalenceTolerance);

        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = end_time - start_time;

        //ROS_INFO("ropeRRTtether took %f seconds", duration.count());



        // Check if the optimized path is valid
        bool pathValid = true;
        

        /**/
        for (size_t i = 0; i < path1.getStateCount(); ++i)
        {   
           // ROS_WARN("checking vaklidty state %zu", i);

            if (!isStateValid(path1.getState(i)))
            {
                pathValid = false;
               // ROS_WARN("Optimized path goes through an obstacle at state %zu", i);
                break;
            }
        }
       

        // Publish the optimized path
        //ROS_INFO("Publishing optimized path...");
        std_msgs::ColorRGBA ropepathColor;
        ropepathColor.r = 0.0f;  // Red
        ropepathColor.g = 1.0f;  // Green
        ropepathColor.b = 0.0f;  // Blue
        ropepathColor.a = 1.0f;  // Alpha (transparency)



        

        
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
        //publishObstacles(obstacle_pub, obstacles, "world");
        publishPointCloud(point_cloud_pub, cloud);
        publishVoxelGrid(voxel_grid_pub, filtered_cloud);
        publishPath(rope_path_pub, path1, "world", "rope_path", ropepathColor);
        publishTetherPath(tether_path_pub, path1, "world");
        publishPath(rov_path_pub, path1, "world", "rov_path", rovpathColor);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}