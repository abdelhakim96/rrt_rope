#include <iostream>
#include <rope_rrt.hpp>
 


int main(int argc, char **argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "rope_shortcut_path_example");
    ros::NodeHandle nh;
    ros::Time ros_time;
    ros::Time last_time = ros::Time::now();  // Track the last time position was updated
 
    ompl::msg::setLogLevel(ompl::msg::LOG_NONE);

    // Create publishers to visualize the original and optimized paths, and obstacles
    ros::Publisher rov_path_pub = nh.advertise<visualization_msgs::Marker>("rov_path", 10);
    ros::Publisher rope_path_pub = nh.advertise<visualization_msgs::Marker>("rope_path", 10);
    ros::Publisher obstacle_pub = nh.advertise<visualization_msgs::Marker>("obstacle", 10);
    ros::Publisher tether_path_pub = nh.advertise<nav_msgs::Path>("`_path", 10);
    ros::Publisher planner_path_pub = nh.advertise<nav_msgs::Path>("planner_path", 10);
    ros::Publisher safe_planner_path_pub = nh.advertise<nav_msgs::Path>("safe_planner_path", 10);
    ros::Publisher direct_path_pub = nh.advertise<visualization_msgs::Marker>("direct_optimal_path", 10);
    ros::Publisher safe_path_pub = nh.advertise<visualization_msgs::Marker>("safe_path", 10);
    ros::Publisher point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("point_cloud", 1);
    ros::Publisher voxel_grid_pub = nh.advertise<sensor_msgs::PointCloud2>("voxel_grid", 1);
    ros::Publisher blue_rov_pub = nh.advertise<visualization_msgs::Marker>("blue_rov", 10);
    ros::Publisher trajectory_pub = nh.advertise<visualization_msgs::Marker>("inspection_reference_trajectory", 10);
    ros::Publisher ref_pub = nh.advertise<geometry_msgs::PoseStamped>("/ropeplanner_goal", 10);
    ros::Publisher cylinder_pub = nh.advertise<visualization_msgs::MarkerArray>("cylinders", 10);
    ros::Publisher exit_points_pub = nh.advertise<visualization_msgs::MarkerArray>("exit_points", 1);

    // Create subscribers
    ros::Subscriber pos_sub = nh.subscribe<nav_msgs::Odometry>("/mobula/rov/odometry", 1, pos_cb);
    ros::Subscriber orientation_sub = nh.subscribe<geometry_msgs::Vector3Stamped>("/mobula/rov/orientation", 1, orientation_cb);
    ros::Subscriber reset_tether_sub = nh.subscribe("reset_tether_topic", 10, reset_tether_cb);
    ros::Subscriber record_trajectory_sub = nh.subscribe("record_trajectory_topic", 10, record_trajectory_on_cb);
    ros::Subscriber goal_point_sub = nh.subscribe<geometry_msgs::PointStamped>("goal_point_pub", 10, goal_cb);

 
 
    // Define obstacles (center and radius)
    ros_time = ros::Time::now();
    ros::Rate rate(20);

    // Load parameters from the parameter server
    nh.param("tether/L_max", L_max, 10.0);
    nh.param("tether_color/r", tetherColor.r, 1.0f);
    nh.param("tether_color/g", tetherColor.g, 0.0f);
    nh.param("tether_color/b", tetherColor.b, 0.0f);
    nh.param("tether_color/a", tetherColor.a, 1.0f);
    nh.param("direct_path_color/r", directPath.r, 1.0f);
    nh.param("direct_path_color/g", directPath.g, 0.0f);
    nh.param("direct_path_color/b", directPath.b, 0.0f);
    nh.param("direct_path_color/a", directPath.a, 1.0f);
    nh.param("safe_path_color/r", safePath.r, 0.0f);
    nh.param("safe_path_color/g", safePath.g, 0.0f);
    nh.param("safe_path_color/b", safePath.b, 1.0f);
    nh.param("safe_path_color/a", safePath.a, 1.0f);
    nh.param("ropepath_color/r", ropepathColor.r, 0.6f);
    nh.param("ropepath_color/g", ropepathColor.g, 0.6f);
    nh.param("ropepath_color/b", ropepathColor.b, 0.0f);
    nh.param("ropepath_color/a", ropepathColor.a, 1.0f);
    nh.param("rovpath_color/r", rovpathColor.r, 0.8f);
    nh.param("rovpath_color/g", rovpathColor.g, 0.8f);
    nh.param("rovpath_color/b", rovpathColor.b, 0.0f);
    nh.param("rovpath_color/a", rovpathColor.a, 1.0f);
    nh.param("safepath_color/r", safepathColor.r, 0.8f);
    nh.param("safepath_color/g", safepathColor.g, 0.8f);
    nh.param("safepath_color/b", safepathColor.b, 0.0f);
    nh.param("safepath_color/a", safepathColor.a, 1.0f);
    nh.param("simulation/time_step", time_step, 1.0);
    nh.param("simulation/ta_planner_on", TA_Planner_ON, true);
    nh.param("tether/delta", delta, 0.2);
    nh.param("tether/eq_tolerance", equivalenceTolerance, 0.000001);
    nh.param("tether/safe_offset", safe_offset, 0.3);
   

   way_point = {0.0, 0.0, 0.0};

    // add cyllinders 
    cylinder_obs cylinder2;
    cylinder2.baseCenter = Eigen::Vector3f(-2.0, 1.0, -1.5);
    cylinder2.axis = Eigen::Vector3f(1.0, -0.0, 0.01); // Assuming the cylinder is aligned with the z-axis
    cylinder2.radius = 0.4;
    cylinder2.height = 5.0;
    cylinders.push_back(cylinder2);
    
    
    cylinder_obs cylinder1;
    cylinder1.baseCenter = Eigen::Vector3f(1.2, 1.0, -1.0);
    cylinder1.axis = Eigen::Vector3f(0.0, 0.0, 1.0); //  Assuming the cylinder is aligned with the z-axis
    cylinder1.radius = 0.4;
    cylinder1.height = 5.0;
    cylinders.push_back(cylinder1);
    
    
    cylinder_obs cylinder2_safe = cylinder2;
    cylinder2_safe.radius += 0.5; // Inflate the radius by 0.3
    cylinders_safe.push_back(cylinder2_safe);
    
    cylinder_obs cylinder1_safe = cylinder1;
    cylinder1_safe.radius += 0.5; // Inflate the radius by 0.3
    cylinders_safe.push_back(cylinder1_safe);


   

    //pcl
     float scale_factor = 0.1; // Scale down by 10 times (0.2 * 0.5)
     Eigen::Vector3f translation(2.0, 1.0, -4.0); // Move closer to the origin
       
       
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
        ompl::geometric::PathGeometric P_t_safe(si);   //Tether safe Path

        ompl::geometric::PathGeometric P_rg(si);  // Path from ROV to Goal (Path option 1: go directly to goal)
        ompl::geometric::PathGeometric iP_t(si);  // Reverse Tether Path
        ompl::geometric::PathGeometric P_t_conc_P_rg(si);  // Tether path concatenated with Path from ROV to Goal
        ompl::geometric::PathGeometric rope_P_t_conc_P_rg(si);  // Tighetened tether path concatenated with Path from ROV to Goal
        ompl::geometric::PathGeometric P_bg(si);  // Shortest path from base to goal (Path option 2: Take long-safe route to goal)
        ompl::geometric::PathGeometric rope_iP_t_conc_P_bt(si);  // Tighetened reverse tether path concatenated with Path from base to Goal
        ompl::geometric::PathGeometric iP_t_conc_P_bg(si);  // Reverse tether path concatenated with Path from base to Goal
        
        
        //define contact points
        std::vector<ompl::base::State *> contactPoints;
        contactPoints.reserve(2000);
        // Define the base position
        std::vector<double> base = {0.0, 0.0, 0.0};
 
 
 
        auto *state_base = si->allocState()->as<ompl::base::RealVectorStateSpace::StateType>();
 
        state_base->values[0] = base[0];  // x-coordinate
        state_base->values[1] = base[1];  // y-coordinate
        state_base->values[2] = base[2];  // z-coordinate
         
        P_t.append(state_base);  
        P_t_safe.append(state_base);


 
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
 




   TetherPlanner planner(delta, equivalenceTolerance); // Create an instance of the TetherPlanner class
   count = 0;

 
    while (ros::ok())
    {
         //initializeVoxelGridAndKdTree(cloud);
       
         auto space = std::make_shared<ompl::base::RealVectorStateSpace>(3);
        ompl::base::RealVectorBounds bounds(3);
        bounds.setLow(-50);
        bounds.setHigh(50);
        space->setBounds(bounds);
 
 
        si->setStateValidityChecker(isStateValid_safe);
 
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
 
        P_t_safe.append(state_rov);

       

         /////////////////////
        //COMPUTE TETHER MODEL
        //////////////////////
        ompl::geometric::PathSimplifier simplifier(si_t);
        ///double delta = 0.2;                // Step size
        //double equivalenceTolerance = 0.000001;  // Equivalence tolerance
       
        // Measure the time taken by the ropeRRTtether method
        auto start_time = std::chrono::high_resolution_clock::now();
 
        //calculate tether P_t
        bool tether_computed = simplifier.ropeRRTtether(P_t, contactPoints, delta, equivalenceTolerance);
        //bool improved = simplifier.ropeShortcutPath(P_t, delta, equivalenceTolerance);
        auto end_time = std::chrono::high_resolution_clock::now();
        time_tether_model_computation = end_time - start_time;
       


         /////////////////////
        //COMPUTE SAFE-TETHER -BASED PATH
        //////////////////////
        //ompl::geometric::PathSimplifier simplifier_safe(si);
       
        //bool tether_safe_computed = simplifier_safe.ropeRRTtether(P_t_safe, contactPoints, delta, equivalenceTolerance);
       




        
                 
       //////////////////
       //GlOBAL PlANNER
    /////////////////////

       
        // go to the next way point (TODO make a function)
        if (count > 0 && count < way_point_traj.size() ) {
            way_point = way_point_traj[count];

        }

        if (distance(way_point, current_pos_att) < 0.1
        && count < way_point_traj.size() - 1 ) {
             ROS_INFO("Waypoint reached:");
            goal_reached = true;
            count++;
        }
        
        
        ROS_INFO("Distance to waypoint: %f", distance(way_point, current_pos_att));

      
      
       ///////////
       //PlANNER
       ///////////
      // if (Tether_Length_exceeded ==true)
      //   {
        //      ROS_INFO("Tether length exceeded");
              //P_t = planner.findNextGoal(P_t, current_pos_att, way_point, L_max, space, si);
            //   P_t = planner.ReplanPath(P_t, current_pos_att, way_point, L_max, space, si);
           
     
       ompl::geometric::PathGeometric Path_sample(si);
       ompl::geometric::PathGeometric Path_safe(si);
       double Tether_length = planner.findTetherLength(P_t);
       ROS_INFO("Tether length is %f",Tether_length );

      
           Path_sample = planner.SearchAlternativePath(P_t, way_point, si_t , L_max); 
           
          
          
           //ompl::geometric::PathGeometric safe_path = planner.OffsetPath(Path_sample , si, safe_offset); 
           //ompl::geometric::PathSimplifier simplify_safe(si);
           //bool simplify_safe_path = simplify_safe.ropeRRTtether(safe_path , contactPoints, delta, equivalenceTolerance);



           // Print the states in the alternative path
           for (std::size_t i = 0; i < Path_sample.getStateCount(); ++i)
           {
               const auto *state = Path_sample.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
              //ROS_INFO("State %zu: [x: %f, y: %f, z: %f]", i, state->values[0], state->values[1], state->values[2]);
           }

 
           if (TA_Planner_ON == true)
           {
             ROS_INFO("Taking Aternative path");
                ROS_INFO("Getting next point along alternative path");
                // way_point = planner.GetNextPointAlongPath(Path_sample,current_pos_att, way_point,si);
                // way_point = planner.GetNextPointAlongPath(safe_path,current_pos_att, way_point,si);                 
                //way_point = planner.GetNextPointAlongPath(Path_sample,current_pos_att, way_point,si);
            // Print the entire path
            ROS_INFO("Alternative Path:");
            for (std::size_t i = 0; i < Path_sample.getStateCount(); ++i)
            {
                const auto *state = Path_sample.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
                ROS_INFO("State %zu: [x: %f, y: %f, z: %f]", i, state->values[0], state->values[1], state->values[2]);
            }

           }
       //}
       
    

        //////////    
        // Publish 
        //////////
        //publishObstacles(obstacle_pub, obstacles, "world");
        publishPointCloud(point_cloud_pub, cloud);
        publishVoxelGrid(voxel_grid_pub, filtered_cloud);
        publishPath(rope_path_pub, P_t, "world", "rope_path", ropepathColor);
        publishTetherPath(tether_path_pub, P_t, "world", tetherColor);       
        publishTrajectory(trajectory_pub, way_point_traj);   
        publishRef(ref_pub, way_point);
        publishCylinders(cylinder_pub, cylinders, "world");
        publishBlueRovMarker(blue_rov_pub, current_pos_att, angles, "world");
        publishTetherPath(planner_path_pub, Path_sample, "world", rovpathColor);
        //publishTetherPath(safe_planner_path_pub, safe_path, "world", safepathColor);




 
        ros::spinOnce();
        rate.sleep();
    }
 
    return 0;
}