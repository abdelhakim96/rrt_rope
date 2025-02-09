#include <rope_rrt.hpp>
 
// Structure to represent an obstacle as a sphere
 

 
 
 


// Helper function to check if a point is inside a cylinder with an arbitrary axis


 
 
 
 
 
 
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
 
 
/*
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

    // Check collision with each cylinder
    for (const auto &cylinder : cylinders)
    {
        // Vector from base center to robot position
        Eigen::Vector3f baseToRobot = robot_position - cylinder.baseCenter;

        // Project the vector onto the cylinder axis
        float projectionLength = baseToRobot.dot(cylinder.axis);
        if (projectionLength < 0 || projectionLength > cylinder.height)
        {
            continue; // Outside the height of the cylinder
        }

        // Calculate the closest point on the cylinder axis
        Eigen::Vector3f closestPoint = cylinder.baseCenter + projectionLength * cylinder.axis;

        // Calculate the distance from the robot position to the closest point
        float distance = (robot_position - closestPoint).norm();
        if (distance < cylinder.radius)
        {
            // ROS_WARN("Collision detected with cylinder.");
            return false; // Collision detected
        }
    }

    // ROS_INFO("State is valid.");
    return true; // No collision
}
*/


/*
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
            ROS_WARN("Collision detected with cylinder.");
            return false; // Collision detected with cylinder
        }
    }

    // ROS_INFO("State is valid.");
    return true; // No collision
}
*/

/*
bool isStateValid(const ompl::base::State *state)
{
    if (!voxel_grid_initialized)
    {
       // ROS_ERROR("Voxel grid not initialized.");
        return false;
    }
 
    // Cast the state to RealVectorStateSpace::StateType
    const auto *realState = state->as<ompl::base::RealVectorStateSpace::StateType>();
    if (!realState)
    {
        //ROS_ERROR("State is not of type RealVectorStateSpace::StateType.");
        return false;
    }
 
    // Access the position values
    double x = realState->values[0];
    double y = realState->values[1];
    double z = realState->values[2];
 
    // Convert position to Eigen vector
    Eigen::Vector3f robot_position(x, y, z);
 
    // Print the robot's position
    //OS_INFO("Checking state at position: [%f, %f, %f]", x, y, z);
 
    // Define a collision threshold delta
    float collision_threshold = 0.2; // Adjust as needed
 
    // Check if the robot's position falls within an occupied voxel using k-d tree
    pcl::PointXYZ search_point(robot_position[0], robot_position[1], robot_position[2]);
    std::vector<int> indices(1);
    std::vector<float> distances(1);
    if (kdtree.nearestKSearch(search_point, 1, indices, distances) > 0 && distances[0] < collision_threshold)
    {
       // ROS_WARN("Collision detected with point at distance %f", distances[0]);
        return false; // Collision detected
    }
 
    //ROS_INFO("State is valid.");
    return true; // No collision
}
 */


 
// Custom motion validator class

/*
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
 
 */
 
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
    ros::Time last_time = ros::Time::now();  // Track the last time position was updated
 
 
    int count = 0;
    // Create publishers to visualize the original and optimized paths, and obstacles
    ros::Publisher rov_path_pub = nh.advertise<visualization_msgs::Marker>("rov_path", 10);
    ros::Publisher rope_path_pub = nh.advertise<visualization_msgs::Marker>("rope_path", 10);
    ros::Publisher obstacle_pub = nh.advertise<visualization_msgs::Marker>("obstacle", 10);
 
    ros::Publisher tether_path_pub = nh.advertise<nav_msgs::Path>("rope_rrt_tether_path", 10);
    ros::Publisher direct_path_pub = nh.advertise<visualization_msgs::Marker>("direct_optimal_path", 10);  
    ros::Publisher safe_path_pub = nh.advertise<visualization_msgs::Marker>("safe_path", 10);
 
    ros::Publisher point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("point_cloud", 1);   ;
    ros::Publisher voxel_grid_pub = nh.advertise<sensor_msgs::PointCloud2>("voxel_grid", 1);
 
 
    ros::Publisher trajectory_pub = nh.advertise<visualization_msgs::Marker>("inspection_reference_trajectory", 10);
   
    ros::Publisher ref_pub = nh.advertise<geometry_msgs::PoseStamped>("/ropeplanner_goal", 10);
     ros::Publisher cylinder_pub = nh.advertise<visualization_msgs::MarkerArray>("cylinders", 10);

 
    //subscribers
    ros::Subscriber pos_sub = nh.subscribe<nav_msgs::Odometry>("/mobula/rov/odometry", 1, pos_cb);
    ros::Subscriber orientation_sub = nh.subscribe<geometry_msgs::Vector3Stamped>("/mobula/rov/orientation", 1, orientation_cb);
    ros::Subscriber goal_point_sub = nh.subscribe<geometry_msgs::PointStamped>("goal_point_pub", 10, goal_cb);
 
 
    // Define obstacles (center and radius)
    ros_time = ros::Time::now();
    ros::Rate rate(100);
    


    // add cyllinders 
    cylinder_obs cylinder2;
    cylinder2.baseCenter = Eigen::Vector3f(1.7, 1.0, -1.5);
    cylinder2.axis = Eigen::Vector3f(1.0, -0.0, 0.01); // Assuming the cylinder is aligned with the z-axis
    cylinder2.radius = 0.8;
    cylinder2.height = 6.0;
    cylinders.push_back(cylinder2);
    
    
    cylinder_obs cylinder1;
    cylinder1.baseCenter = Eigen::Vector3f(5.5, 1.0, 1.0);
    cylinder1.axis = Eigen::Vector3f(0.0, 0.0, 1.0); // Assuming the cylinder is aligned with the z-axis
    cylinder1.radius = 0.7;
    cylinder1.height = 7.0;
    cylinders.push_back(cylinder1);

    



   //pcl
     float scale_factor = 0.1; // Scale down by 10 times (0.2 * 0.5)
        Eigen::Vector3f translation(-2.0, 1.0, -4.0); // Move closer to the origin
       
       
        //Eigen::Vector3f translation(2.0, 1.0, 1.0); // Move closer to the origin
 
        Eigen::Matrix3f rotation;
        rotation = Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitX());
        std::string filename = "/home/hakim/tether_planning_ws/src/rope_rrt/pipe.pcd";
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1)
            {
                PCL_ERROR("Couldn't read file %s \n", filename.c_str());
                return -1;
            }
 
       densifyPointCloud(cloud);
 
 
        transformPointCloud(cloud, scale_factor, translation, rotation);
 
 
      Eigen::Vector3f translation1(0, 0, 0);  // Set translation to zero
 
       Eigen::Matrix3f rotation1;
        rotation1 = Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitY());  // Rotate by 90 degrees around X-axis
 
 
 
        transformPointCloud(cloud, 1.0, translation1, rotation1);
 
       //voxel grid
 
        initializeVoxelGridAndKdTree(cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);  
        //transformPointCloud(filtered_cloud, scale_factor, translation, rotation);
        voxel_grid.filter(*filtered_cloud);
 

 
       //create space and path
 
 
        auto space = std::make_shared<ompl::base::RealVectorStateSpace>(3);
        ompl::base::RealVectorBounds bounds(3);
        bounds.setLow(-30);
        bounds.setHigh(30);
        space->setBounds(bounds);
       
       // Create space information and set the state validity checker
        auto si = std::make_shared<ompl::base::SpaceInformation>(space);
 
        si->setStateValidityChecker(isStateValid);
 
        // Set the custom motion validator
        si->setMotionValidator(std::make_shared<CustomMotionValidator>(si));
        si->setup();
 
 
 
        auto si_t = std::make_shared<ompl::base::SpaceInformation>(space);
 
        si_t->setStateValidityChecker(isStateValid);
 
        // Set the custom motion validator
        si_t->setMotionValidator(std::make_shared<CustomMotionValidator>(si_t));
        si_t->setup();
 
        // Define a geometric path in the space (Semi-Circular Path)
        ompl::geometric::PathGeometric P_t(si_t);   //Tether Path
        ompl::geometric::PathGeometric P_rg(si);  // Path from ROV to Goal (Path option 1: go directly to goal)
        ompl::geometric::PathGeometric iP_t(si);  // Reverse Tether Path
        ompl::geometric::PathGeometric P_t_conc_P_rg(si);  // Tether path concatenated with Path from ROV to Goal
        ompl::geometric::PathGeometric rope_P_t_conc_P_rg(si);  // Tighetened tether path concatenated with Path from ROV to Goal
        ompl::geometric::PathGeometric P_bg(si);  // Shortest path from base to goal (Path option 2: Take long-safe route to goal)
        ompl::geometric::PathGeometric rope_iP_t_conc_P_bt(si);  // Tighetened reverse tether path concatenated with Path from base to Goal
        ompl::geometric::PathGeometric iP_t_conc_P_bg(si);  // Reverse tether path concatenated with Path from base to Goal
        //define contact points
        std::vector<ompl::base::State *> contactPoints;
       
        // Define the base position
        std::vector<double> base = {0.0, 0.0, 0.0};
 
 
 
        auto *state_base = si->allocState()->as<ompl::base::RealVectorStateSpace::StateType>();
 
        state_base->values[0] = base[0];  // x-coordinate
        state_base->values[1] = base[1];  // y-coordinate
        state_base->values[2] = base[2];  // z-coordinate
         
        // P_t.append(state_base);
       
 
 
    std::string filePath = "/home/hakim/tether_planning_ws/src/ea_mpc/pipe_traj.txt"; // Provide the file path
    double distanceThreshold = 10.0;  // Distance threshold for sparsification (meters)
   
    std::vector<std::vector<double>> way_point_traj = sparsifyTrajectory(filePath, distanceThreshold);
 
 
    rotation = Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitX());  // Rotate by 90 degrees around X-axis
 
   
    transformWaypoints(way_point_traj, scale_factor, translation, rotation);
 
    translation.setZero();  // Set translation to zero
 
    rotation = Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitY());  // Rotate by 90 degrees around X-axis
 
    transformWaypoints(way_point_traj, 1.0, translation, rotation);
 
 
 
   for (int i = 0; i < 10; ++i) {
    ros::spinOnce();
    rate.sleep();
}
 






 
    while (ros::ok())
    {
         initializeVoxelGridAndKdTree(cloud);
       
         auto space = std::make_shared<ompl::base::RealVectorStateSpace>(3);
        ompl::base::RealVectorBounds bounds(3);
        bounds.setLow(-50);
        bounds.setHigh(50);
        space->setBounds(bounds);
 
 
        si->setStateValidityChecker(isStateValid);
 
        // Set the custom motion validator
         si->setMotionValidator(std::make_shared<CustomMotionValidator>(si));
         si->setup();
 
 
        si_t->setStateValidityChecker(isStateValid);
 
        // Set the custom motion validator
         si_t->setMotionValidator(std::make_shared<CustomMotionValidator>(si_t));
         si_t->setup();
        auto *state_rov = si_t->allocState()->as<ompl::base::RealVectorStateSpace::StateType>();
 
        state_rov->values[0] = current_pos_att[0];  // x-coordinate
        state_rov->values[1] = current_pos_att[1];  // y-coordinate
         state_rov->values[2] = current_pos_att[2];  // z-coordinate
 
        P_t.append(state_rov);
 
 
       
 
 
 
   
 
        // Publish the original path
        std_msgs::ColorRGBA rovpathColor;
        rovpathColor.r = 0.0f;  // Red
        rovpathColor.g = 0.0f;  // Green
        rovpathColor.b = 1.0f;  // Blue
        rovpathColor.a = 1.0f;  // Alpha (transparency)
 
 
        std_msgs::ColorRGBA directpathColor;
        directpathColor.r = 1.0f;  // Red
        directpathColor.g = 0.0f;  // Green
        directpathColor.b = 0.0f;  // Blue
        directpathColor.a = 1.0f;  // Alpha (transparency)
 
 
        std_msgs::ColorRGBA safepathColor;
         safepathColor.r = 0.0f;  // Red
         safepathColor.g = 1.0f;  // Green
         safepathColor.b = 1.0f;  // Blue
         safepathColor.a = 1.0f;  // Alpha (transparency)
 
 
        // Simplify the path using ropeShortcutPath (Optimized Path)
        ompl::geometric::PathSimplifier simplifier(si);
        double delta = 1.0;                // Step size
        double equivalenceTolerance = 0.000001;  // Equivalence tolerance
       
        // Measure the time taken by the ropeRRTtether method
        auto start_time = std::chrono::high_resolution_clock::now();
 
        //calculate tether P_t
        bool improved = simplifier.ropeRRTtether(P_t, contactPoints, delta, equivalenceTolerance);
        //bool improved = simplifier.ropeShortcutPath(P_t, delta, equivalenceTolerance);
 
 
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = end_time - start_time;
       
        iP_t.clear();
        // Calculate the reverse tether path
        for (int i = P_t.getStateCount() - 1; i >= 0; --i)
        {
             // Clone the state to ensure a new instance is created
             auto *state = P_t.getSpaceInformation()->cloneState(P_t.getState(i));
               iP_t.append(state);
        }
       
         
        // calculate P_bg
       
       
        // Check if goal has been updated
        //////////////////
        //Path Planner
        ////////////////////        ////////////////////        ////////////////////        ////////////////////        ////////////////////        ////////////////////        ////////////////////        ////////////////////        ////////////////////        ////////////////////        ////////////////////        ////////////////////        ////////////////////        ////////////////////        ////////////////////        ////////////////////
       
        /*
       
        if (goal_updated(goal, goal_t1, 0.01))
        {  
            std::cout<<"goal updated"<<std::endl;
            ompl::base::ScopedState<ompl::base::RealVectorStateSpace> state_goal(space);
 
        // Reset the paths
            P_rg.clear();
            P_bg.clear();
            iP_t_conc_P_bg.clear();
            rope_P_t_conc_P_rg.clear();
            P_t_conc_P_rg.clear();
 
            state_goal->values[0] = goal[0];  // x-coordinate
            state_goal->values[1] = goal[1];  // y-coordinate
            state_goal->values[2] = goal[2];  // z-coordinate
 
            ompl::base::ScopedState<ompl::base::RealVectorStateSpace> state_rov_scoped(space);
            state_rov_scoped->values[0] = current_pos_att[0];  // x-coordinate
            state_rov_scoped->values[1] = current_pos_att[1];  // y-coordinate
            state_rov_scoped->values[2] = current_pos_att[2];  // z-coordinate
 
            ompl::base::ScopedState<ompl::base::RealVectorStateSpace> state_base_scoped(space);
            state_base_scoped->values[0] = base[0];  // x-coordinate
            state_base_scoped->values[1] = base[1];  // y-coordinate
            state_base_scoped->values[2] = base[2];  // z-coordinate
 
            // Calculate the path from the base to the goal using RRT*
            ompl::geometric::SimpleSetup ss_rg(si);
            ss_rg.setStartAndGoalStates(state_rov_scoped, state_goal);
            auto planner_rg = std::make_shared<ompl::geometric::RRTstar>(si);
            ss_rg.setPlanner(planner_rg);
 
            // Solve the planning problem
            ompl::base::PlannerStatus solved_rg = ss_rg.solve(ompl::base::timedPlannerTerminationCondition(1.0));
 
            // Calculate the path from the base to the goal using RRT*
            ompl::geometric::SimpleSetup ss_bg(si);
            ss_bg.setStartAndGoalStates(state_base_scoped, state_goal);
            auto planner_bg = std::make_shared<ompl::geometric::RRTstar>(si);
            ss_bg.setPlanner(planner_bg);
 
            // Solve the planning problem for P_bg
            ompl::base::PlannerStatus solved_bg = ss_bg.solve(ompl::base::timedPlannerTerminationCondition(1.0));
 
            if (solved_rg && solved_bg)
            {
                // Get the solution paths
                P_rg = ss_rg.getSolutionPath();
                P_bg = ss_bg.getSolutionPath();
 
                // Optionally, simplify the solution paths
                ss_rg.simplifySolution();
                P_rg = ss_rg.getSolutionPath();
 
                ss_bg.simplifySolution();
                P_bg = ss_bg.getSolutionPath();
 
                // Concatenate P_t with P_rg
                ompl::geometric::PathGeometric P_t_conc_P_rg = P_t;
                P_t_conc_P_rg.append(P_rg);
                ompl::geometric::PathGeometric rope_P_t_conc_P_rg = P_t_conc_P_rg;
                bool improved_P_t_conc_P_rg = simplifier.ropeRRTtether(rope_P_t_conc_P_rg, contactPoints, delta, equivalenceTolerance);
 
                // Concatenate iP_t with P_bg
                iP_t_conc_P_bg = iP_t;
                iP_t_conc_P_bg.append(P_bg);
                ompl::geometric::PathGeometric rope_iP_t_conc_P_bg = iP_t_conc_P_bg;
                bool improved_iP_t_conc_P_bg = simplifier.ropeRRTtether(rope_iP_t_conc_P_bg, contactPoints, delta, equivalenceTolerance);
            }
            else
            {
                if (!solved_rg)
                {
                    std::cout << "No solution found for P_rg." << std::endl;
                }
                if (!solved_bg)
                {
                    std::cout << "No solution found for P_bg." << std::endl;
                }
            }
        }
        */
       
        //ROS_INFO("ropeRRTtether took %f seconds", duration.count());
 
 
 
 
 
 
      if ((ros::Time::now() - last_time) >= ros::Duration(0.5))  // 0.1 second passed
            {  
                goal_t1 = goal;  // Store the position 0.1 seconds before
                last_time = ros::Time::now();  // Update the last time to the current time
            }  
 
 
 
 
        // Check if the optimized path is valid
        bool pathValid = true;
       
 
        /**/
        for (size_t i = 0; i < P_t.getStateCount(); ++i)
        {  
           // ROS_WARN("checking vaklidty state %zu", i);
 
            if (!isStateValid(P_t.getState(i)))
            {
                pathValid = false;
               // ROS_WARN("Optimized path goes through an obstacle at state %zu", i);
                break;
            }
        }
       
 
        // Publish the optimized path
        //ROS_INFO("Publishing optimized path...");
        std_msgs::ColorRGBA ropepathColor;
        ropepathColor.r = 0.6f;  // Red
        ropepathColor.g = 0.6f;  // Green
        ropepathColor.b = 0.0f;  // Blue
        ropepathColor.a = 1.0f;  // Alpha (transparency)
 
 
 
       
        /*
       
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
       */
        // Publish the obstacles
        //publishObstacles(obstacle_pub, obstacles, "world");
        publishPointCloud(point_cloud_pub, cloud);
        publishVoxelGrid(voxel_grid_pub, filtered_cloud);
        publishPath(rope_path_pub, P_t, "world", "rope_path", ropepathColor);
       
        publishPath(rov_path_pub, P_t, "world", "rov_path", rovpathColor);
 
        if (P_rg.getStateCount() > 0)
        {
            ROS_INFO("Path P_rg:");
            for (size_t i = 0; i < P_rg.getStateCount(); ++i)
            {
                const auto *state = P_rg.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
                ROS_INFO("State %zu: [x: %f, y: %f, z: %f]", i, state->values[0], state->values[1], state->values[2]);
            }
        }
 
 
 
 
 
        std_msgs::ColorRGBA tetherColor;
        tetherColor.r = 1.0f;  // Red
        tetherColor.g = 0.0f;  // Green
        tetherColor.b = 0.0f;  // Blue
        tetherColor.a = 1.0f;  // Alpha (transparency)
 
 
        std_msgs::ColorRGBA direct_path;
        direct_path.r = 1.0f;  // Red
        direct_path.g = 0.0f;  // Green
        direct_path.b = 0.0f;  // Blue
        direct_path.a = 1.0f;  // Alpha (transparency)
 
        std_msgs::ColorRGBA safe_path;
        safe_path.r = 0.0f;  // Red
        safe_path.g = 0.0f;  // Green
        safe_path.b = 1.0f;  // Blue
        safe_path.a = 1.0f;  // Alpha (transparency)
 
 
 
        publishTetherPath(tether_path_pub, P_t, "world", tetherColor);
 
        publishTetherPath(direct_path_pub, P_rg, "world",  direct_path);
        publishTetherPath(safe_path_pub, iP_t_conc_P_bg, "world", safe_path);
       
        publishTrajectory(trajectory_pub, way_point_traj);
       
        std::vector<double> way_point = {0.0, 0.0, 0.0, 0.0};  // Initialize with four elements
 
        if (count > 0 && count < way_point_traj.size()) {
            way_point = way_point_traj[count];
        }
 
        if (distance(way_point, current_pos_att) < 0.05 && count < way_point_traj.size() - 1) {
 
            // if (count == 0) {
        ROS_INFO("Current position and attitude when count is 0: [x: %f, y: %f, z: %f, yaw: %f]",
                 current_pos_att[0], current_pos_att[1], current_pos_att[2], current_pos_att[3]);
    //}
            count++;
        }
               
       
        publishRef(ref_pub, way_point);
        publishCylinders(cylinder_pub, cylinders, "world");

        //publishPath(direct_path_pub, P_rg, "world", "direct_optimal_path", directpathColor);
        //publishPath(safe_path_pub, iP_t_conc_P_bg, "world", "safe_path", safepathColor);
       // if (P_rg.getStateCount() == 0)
       // {
           // ROS_ERROR("No solution found for P_rg. Goal x-coordinate: %f", goal[0]);
 
        //}
 
        ros::spinOnce();
        rate.sleep();
    }
 
    return 0;
}