
#include "tether_planner.hpp"



TetherPlanner::TetherPlanner()
{
   //Constructor
   std::cout<<"Tether Planner Initialized"<<std::endl;  

}



void TetherPlanner::findNextGoal(const ompl::geometric::PathGeometric &tether , 
                const std::vector<double> &current_position, 
                const std::vector<double> &goal, 
                double &L_max,         
                const std::shared_ptr<ompl::base::RealVectorStateSpace> &space,
                const std::shared_ptr<ompl::base::SpaceInformation> &si)
{
    ompl::base::RealVectorBounds bounds(3);
    bounds.setLow(-30);
    bounds.setHigh(30);
    std::vector<double> point_exit(3); // Change to std::vector<double> of size 3

    ompl::geometric::PathGeometric Path_ref(si);
    ompl::geometric::SimpleSetup ss_rrt(si);
    bool DENT = 0;
    double L = findTetherLength(tether);

    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> state_goal(space);
    state_goal->values[0] = goal[0];  // x-coordinate
    state_goal->values[1] = goal[1];  // y-coordinate
    state_goal->values[2] = goal[2];  // z-coordinate

    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> state_rov_scoped(space);
    state_rov_scoped->values[0] = current_position[0];  // x-coordinate
    state_rov_scoped->values[1] = current_position[1];  // y-coordinate
    state_rov_scoped->values[2] = current_position[2];  // z-coordinate
     ompl::base::PlannerStatus solved_rrt;

   
   if (L >L_max & DENT == 0)
   {
     //point_exit = findExitPoint(tether);
     Path_ref = InvertTetherPath(tether, si);
     DENT = 1;
    }
    
    else if( L < L_max && DENT == 1 && isEqual(current_position, point_exit))
    {      
      ss_rrt.setStartAndGoalStates(state_rov_scoped, state_goal);
      auto planner_rg = std::make_shared<ompl::geometric::RRTstar>(si);
      ss_rrt.setPlanner(planner_rg);
 
            // Solve the planning problem
      solved_rrt = ss_rrt.solve(ompl::base::timedPlannerTerminationCondition(1.0));
      Path_ref = ss_rrt.getSolutionPath();
      DENT = 0;
    }  
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



bool TetherPlanner::isEqual(const std::vector<double> &point1, const std::vector<double> &point2) const
{
    const double epsilon = 1e-3; // Tolerance for floating-point comparison
    return (std::abs(point1[0] - point2[0]) < epsilon &&
            std::abs(point1[1] - point2[1]) < epsilon &&
            std::abs(point1[2] - point2[2]) < epsilon);
}