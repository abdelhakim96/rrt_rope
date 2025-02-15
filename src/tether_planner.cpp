
#include "tether_planner.hpp"
#include <boost/type_traits/is_reference.hpp>
#include <filesystem>
#include <ompl/geometric/PathGeometric.h>
#include <vector>



TetherPlanner::TetherPlanner()
{
   //Constructor
   std::cout<<"Tether Planner Initialized"<<std::endl;  

}



ompl::geometric::PathGeometric TetherPlanner::findNextGoal(const ompl::geometric::PathGeometric &tether , 
                const std::vector<double> &current_position, 
                const std::vector<double> &goal, 
                double &L_max,         
                const std::shared_ptr<ompl::base::RealVectorStateSpace> &space,
                const std::shared_ptr<ompl::base::SpaceInformation> &si)
{   

  //init states and bounds

    ompl::base::RealVectorBounds bounds(3);
    bounds.setLow(-30);
    bounds.setHigh(30);
    std::vector<double> point_exit(3); // Change to std::vector<double> of size 3
     
    ompl::geometric::PathGeometric Path_ref(si);
    ompl::geometric::SimpleSetup ss_rrt(si);
    ompl::geometric::PathGeometric Path_replan(si); 
    bool ENT = 0;

    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> state_goal(space);
    state_goal->values[0] = goal[0];  // x-coordinate
    state_goal->values[1] = goal[1];  // y-coordinate
    state_goal->values[2] = goal[2];  // z-coordinate

    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> state_rov_scoped(space);
    state_rov_scoped->values[0] = current_position[0];  // x-coordinate
    state_rov_scoped->values[1] = current_position[1];  // y-coordinate
    state_rov_scoped->values[2] = current_position[2];  // z-coordinate
     ompl::base::PlannerStatus solved_rrt;
    

   
   std::vector<double> exit_point;
   double L = findTetherLength(tether);
   std::vector<double> goal_next;

   std::vector<double> goal_last = goal_next;
      UpdateExitPointsList(tether); 

    std::cout << "Current tether length: " << L << std::endl;
    if (L>0 && exit_points_list_.size() > 0){
    if (L > L_max && ENT == 0 )
    {
        std::cout << "Tether constraint violated. Activating entanglement." << std::endl;
        // Tether constraint violated - activation entanglement, going back along path and find exit point
        UpdateExitPointsList(tether); 
        std::cout << "Updated Exit Points List." << std::endl;
      
        std::cout << "exit_points_list_.size()." << exit_points_list_.size()<< std::endl;

        ENT = 1;
        for (int i = 0; i < exit_points_list_.size(); i++)
        {
            double L_replan_length = findTetherLengthReplannedPath(tether, exit_points_list_[i], goal, space, si);   
            //double L_replan_length = 10.0;
            std::cout << "Replanned tether length: " << L_replan_length << std::endl;
            if (L_replan_length < L_max)
            {
                // Found exit point, replanning path to goal
                std::cout << "Found exit point. Replanning path to goal." << std::endl;
                Path_replan = ReplanPath(tether, exit_points_list_[i], goal, si); 
                point_exit = exit_points_list_[i];
                break;
            }
        }
    }
    else if (L < L_max && ENT == 0)
    {
        std::cout << "Tether constraint not violated. Going directly to goal." << std::endl;
        // Tether constraint not violated - go directly to goal
        Path_ref = FindShortestPath(state_rov_scoped, state_goal, si);
    }
    else if (ENT == 1 && exit_points_list_.size() > 0)
    {
        std::cout << "Following replanned path." << std::endl;
        goal_next = FindNextPointAlongPath(current_position, goal_last, Path_replan);
        if (isEqual(current_position, point_exit))
        {
            std::cout << "Reached exit point. Deactivating entanglement." << std::endl;
            ENT = 0;
        }
    }

    //std::cout << "Next goal: [" << goal_next[0] << ", " << goal_next[1] << ", " << goal_next[2] << "]" << std::endl;
    }


    //std_msgs::ColorRGBA ropepathColor;
    //ropepathColor.r = 0.6f;  // Red
    //ropepathColor.g = 0.6f;  // Green
    //ropepathColor.b = 0.0f;  // Blue
    //ropepathColor.a = 1.0f;  // Alpha (transparency)

    //publishPath(rov_path_pub, Path_replan, "world", "rov_path", ropepathColor);


    return Path_replan;
}


ompl::geometric::PathGeometric TetherPlanner::InvertTetherPath(const ompl::geometric::PathGeometric &tether, 
                                                  const std::shared_ptr<ompl::base::SpaceInformation> &si)
   {

        ompl::geometric::PathGeometric tether_inverse(si);  // Reverse tether path concatenated with Path from base to Goal

        for (int i = tether.getStateCount() - 1; i >= 0; --i)
        {
             // Clone the state to ensure a new instance is created
             auto *state = tether.getSpaceInformation()->cloneState(tether.getState(i));
               tether_inverse.append(state);
        }

    return tether_inverse;
    }






double TetherPlanner::findTetherLength(const ompl::geometric::PathGeometric &path)
{
    double length = 0.0;

    // Debugging print to check the number of states in the path
    //std::cout << "Number of states in path: " << path.getStateCount() << std::endl;

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
        double segment_length = (p2 - p1).norm();
        length += segment_length;

        // Debugging prints
        //std::cout << "State " << i-1 << ": [" << p1[0] << ", p1[1] << ", " << p1[2] << "]" << std::endl;
        ////std::cout << "State " << i << ": [" << p2[0] << ", p2[1] << ", " << p2[2] << "]" << std::endl;
        //std::cout << "Segment length: " << segment_length << std::endl;
    }

    //std::cout << "Total tether length: " << length << std::endl;
    return length;
}



double TetherPlanner::findTetherLengthReplannedPath(const ompl::geometric::PathGeometric &tether, 
                                               const std::vector<double> &exit_point, 
                                                const std::vector<double> &goal,
                                                const std::shared_ptr<ompl::base::RealVectorStateSpace> &space,
                                                 const std::shared_ptr<ompl::base::SpaceInformation> &si)
{
    double length = 0.0;
    // find length till exitpoint
    for (std::size_t i = 1; i < tether.getStateCount(); ++i)
    {   

        
        // Get the current and previous states
        const auto *state1 = tether.getState(i - 1)->as<ompl::base::RealVectorStateSpace::StateType>();
        const auto *state2 = tether.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();


        if (!isEqual({state1->values[0], state1->values[1], state1->values[2]}, exit_point))
        {
            

      
        // Convert the states to Eigen vectors
        Eigen::Vector3d p1(state1->values[0], state1->values[1], state1->values[2]);
        Eigen::Vector3d p2(state2->values[0], state2->values[1], state2->values[2]);

        // Calculate the distance between the states and add to the total length
        length += (p2 - p1).norm();
        }
        else
        {   
            //ompl::geometric::PathGeometric Path_from_exit = FindShortestPath(exit_point, goal, space, si);
            ompl::base::ScopedState<ompl::base::RealVectorStateSpace> state_goal(space);
            state_goal->values[0] = goal[0];  // x-coordinate
            state_goal->values[1] = goal[1];  // y-coordinate
            state_goal->values[2] = goal[2];  // z-coordinate

            ompl::base::ScopedState<ompl::base::RealVectorStateSpace> state_rov_scoped(space);
            state_rov_scoped->values[0] =  current_pos_att[0];  // x-coordinate
            state_rov_scoped->values[1] =  current_pos_att[1];  // y-coordinate
            state_rov_scoped->values[2] =  current_pos_att[2];  // z-coordinate

            ompl::geometric::PathGeometric Path_from_exit = FindShortestPath(state_rov_scoped, state_goal,  si);

            double L_from_exit = findTetherLength(Path_from_exit);
            length += L_from_exit;
            break;
        }
    }
  

    return length;
}





bool TetherPlanner::isEqual(const std::vector<double> &point1, const std::vector<double> &point2) const
{
    const double epsilon = 1e-3; // Tolerance for floating-point comparison
    return (std::abs(point1[0] - point2[0]) < epsilon &&
            std::abs(point1[1] - point2[1]) < epsilon &&
            std::abs(point1[2] - point2[2]) < epsilon);
}




ompl::geometric::PathGeometric TetherPlanner::FindShortestPath(const ompl::base::ScopedState<ompl::base::RealVectorStateSpace> &start,
                                                               const ompl::base::ScopedState<ompl::base::RealVectorStateSpace> &goal,
                                                               const std::shared_ptr<ompl::base::SpaceInformation> &si)
{
    ompl::geometric::SimpleSetup ss_rrt(si);
    ss_rrt.setStartAndGoalStates(start, goal);
    auto planner_rg = std::make_shared<ompl::geometric::RRTstar>(si);
    ss_rrt.setPlanner(planner_rg);

    // Solve the planning problem
    ompl::base::PlannerStatus solved_rrt = ss_rrt.solve(ompl::base::timedPlannerTerminationCondition(1.0));
    if (solved_rrt)
    {
        return ss_rrt.getSolutionPath();
    }
    else
    {
        // Return an empty path if the planning problem is not solved
        return ompl::geometric::PathGeometric(si);
    }
}


void TetherPlanner::UpdateExitPointsList(const ompl::geometric::PathGeometric &tether)
{
    // Update the list of potential exit points for replanning
    exit_points_list_.clear();
    bool in_collision = false;

    for (int i = tether.getStateCount() - 1; i >= 0; --i)
    {
        if (!isStateValid(tether.getState(i))) 
        {
            in_collision = true;
        }
        else if (in_collision)
        {
            // Add the first non-collision point after a collision to the exit points list
            std::vector<double> exit_point(3);
            const auto *state = tether.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
            exit_point[0] = state->values[0];
            exit_point[1] = state->values[1];
            exit_point[2] = state->values[2];
            exit_points_list_.push_back(exit_point);

            // Reset the collision flag to find the next segment
            in_collision = false;
        }
    }
}


std::vector<double> TetherPlanner::FindNextPointAlongPath(const std::vector<double> &current_position, 
                                                          const std::vector<double> &goal_last, 
                                                          const ompl::geometric::PathGeometric &path_replan)
{
    bool found_goal_last = false;

    // Iterate through the path from the end to the beginning
    for (int i = path_replan.getStateCount() - 1; i >= 0; --i)
    {
        const auto *state = path_replan.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
        std::vector<double> point(3);
        point[0] = state->values[0];
        point[1] = state->values[1];
        point[2] = state->values[2];

        // Check if the current position is equal to the last goal point
        if (found_goal_last)
        {
            // Return the next point along the path
            return point;
        }

        // Check if the current point is equal to the last goal point
        if (isEqual(point, goal_last))
        {
            found_goal_last = true;
        }
    }

    // If no next point is found, return the last goal point
    return goal_last;
}



ompl::geometric::PathGeometric TetherPlanner::ReplanPath(const ompl::geometric::PathGeometric &tether, 
                                                         const std::vector<double> &exit_point, 
                                                         const std::vector<double> &goal,
                                                         const std::shared_ptr<ompl::base::SpaceInformation> &si)
{
    ompl::geometric::PathGeometric replan_path(si);

    // Iterate through the tether from end to start
    for (int i = tether.getStateCount() - 1; i >= 0; --i)
    {
        const auto *state = tether.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
        std::vector<double> point(3);
        point[0] = state->values[0];
        point[1] = state->values[1];
        point[2] = state->values[2];

        // Add the state to the replan path
        replan_path.append(state);

        // Check if the current point is equal to the exit point
        if (isEqual(point, exit_point))
        {
            break;
        }
    }

    // Create the shortest path from the exit point to the goal
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> state_exit(si->getStateSpace());
    state_exit->values[0] = exit_point[0];
    state_exit->values[1] = exit_point[1];
    state_exit->values[2] = exit_point[2];

    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> state_goal(si->getStateSpace());
    state_goal->values[0] = goal[0];
    state_goal->values[1] = goal[1];
    state_goal->values[2] = goal[2];

    ompl::geometric::PathGeometric shortest_path = FindShortestPath(state_exit, state_goal, si);

    // Concatenate the replan path with the shortest path to the goal
    replan_path.append(shortest_path);


    ROS_INFO("Replanned Path:");
    for (std::size_t i = 0; i < replan_path.getStateCount(); ++i)
    {
        const auto *state = replan_path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
        ROS_INFO("State %zu: [%f, %f, %f]", i, state->values[0], state->values[1], state->values[2]);
    }



    return replan_path;
}






ompl::geometric::PathGeometric TetherPlanner::CalculateAlternativePath_i(const int node_n, 
    ompl::geometric::PathGeometric tether,
    const std::vector<double> goal,
    const std::shared_ptr<ompl::base::SpaceInformation> &si)
{
ompl::geometric::PathGeometric Alternative_Path(si);
ompl::geometric::PathGeometric Path_segment1(si);
ompl::geometric::PathGeometric Path_segment3(si);

//ompl::geometric::PathGeometric Path_segment2(si);

// Calculate Path_segment1
//ROS_INFO("Calculating Path_segment1...");
for (int i = 0; i < node_n; i++)
{
const auto *state = tether.getState(tether.getStateCount() - i -1 )->as<ompl::base::RealVectorStateSpace::StateType>();
Path_segment1.append(state);
//ROS_INFO("Path_segment1 State %d: [x: %f, y: %f, z: %f]", i, state->values[0], state->values[1], state->values[2]);
}


// Calculate Path_segment3 (the rest of the tether path)
for (int i = tether.getStateCount() - node_n - 1; i >= 0; --i)
{
    const auto *state = tether.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
    Path_segment3.append(state);
}


// Calculate Path_segment2
//ROS_INFO("Calculating Path_segment2...");
auto *state_goal = si->allocState()->as<ompl::base::RealVectorStateSpace::StateType>();
state_goal->values[0] = goal[0];  // x-coordinate
state_goal->values[1] = goal[1];  // y-coordinate
state_goal->values[2] = goal[2];  // z-coordinate
//ROS_INFO("Goal State: [x: %f, y: %f, z: %f]", state_goal->values[0], state_goal->values[1], state_goal->values[2]);

//Path_segment2 = tether;

//ROS_INFO("Calculating Path_segment2...");

/**/
ompl::geometric::PathGeometric Path_segment2 = computePathSegment2(Path_segment1, goal, si);


// Concatenate Path_segment1 and Path_segment2
//ROS_INFO("Concatenating Path_segment1 and Path_segment2...");
Alternative_Path = Path_segment1;
Alternative_Path.append(Path_segment2);
std::vector<ompl::base::State *> contactPoints;
ompl::geometric::PathSimplifier simplifier(si);

simplifier.ropeRRTtether(Alternative_Path,contactPoints, delta_, equivalenceTolerance_);

Alternative_Path_Tether_Length = findTetherLength(Path_segment2)+ findTetherLength(Path_segment3);

// Print the states in the Alternative_Path
//ROS_INFO("Alternative Path:");
for (std::size_t i = 0; i < Alternative_Path.getStateCount(); ++i)
{
const auto *state = Alternative_Path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
//ROS_INFO("State %zu: [x: %f, y: %f, z: %f]", i, state->values[0], state->values[1], state->values[2]);
}

return Alternative_Path;
}





std::vector<double> TetherPlanner::getNextGoal(const ompl::geometric::PathGeometric &path)
{
    std::vector<double> next_goal(3);

    if (path.getStateCount() > 0)
    {
        const auto *state = path.getState(0)->as<ompl::base::RealVectorStateSpace::StateType>();
        next_goal[0] = state->values[0];
        next_goal[1] = state->values[1];
        next_goal[2] = state->values[2];
    }
    else
    {
        ROS_WARN("Path is empty. Returning default goal.");
        next_goal = {0.0, 0.0, 0.0}; // Default goal if path is empty
    }

    ROS_INFO("Next goal: [x: %f, y: %f, z: %f]", next_goal[0], next_goal[1], next_goal[2]);
    return next_goal;
}















ompl::geometric::PathGeometric TetherPlanner::SearchAlternativePath(ompl::geometric::PathGeometric tether,
    const std::vector<double> goal,
    const std::shared_ptr<ompl::base::SpaceInformation> &si,
    const double L_max)
{
ompl::geometric::PathGeometric Alternative_Path(si);
bool short_cut = false;
for (int i = 2; i < tether.getStateCount(); i++)

    {  
        Alternative_Path = CalculateAlternativePath_i(i, tether, goal, si);

        if (Alternative_Path_Tether_Length < L_max)
        {   
        
        ROS_INFO("Found alternative path with length: %f at node number: %d", Alternative_Path_Tether_Length, i);
        short_cut = true;
        break;
        }
    }

if (short_cut = false)
    {
        ROS_WARN("No alternative path found within the tether length constraint. Returning path with single endpoint of tether.");
        const auto *end_state = tether.getState(tether.getStateCount() - 1)->as<ompl::base::RealVectorStateSpace::StateType>();
        ompl::geometric::PathGeometric single_point_path(si);
        single_point_path.append(end_state);
        return single_point_path;
    }

return Alternative_Path;
}




ompl::geometric::PathGeometric TetherPlanner::computePathSegment2(const ompl::geometric::PathGeometric &Path_segment1,
    const std::vector<double> &goal,
    const std::shared_ptr<ompl::base::SpaceInformation> &si)
{
ompl::geometric::PathGeometric Path_segment2 = InvertTetherPath(Path_segment1, si);

auto *state_goal = si->allocState()->as<ompl::base::RealVectorStateSpace::StateType>();
state_goal->values[0] = goal[0];  // x-coordinate
state_goal->values[1] = goal[1];  // y-coordinate
state_goal->values[2] = goal[2];  // z-coordinate

//ROS_INFO("Goal State: [x: %f, y: %f, z: %f]", state_goal->values[0], state_goal->values[1], state_goal->values[2]);


Path_segment2.append(state_goal);

ompl::geometric::PathSimplifier simplifier(si);
simplifier.ropeShortcutPath(Path_segment2, delta_, equivalenceTolerance_);

return Path_segment2;
}