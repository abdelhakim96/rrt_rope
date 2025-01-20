#include <ros/ros.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

// Set the bounds of the planning space
void setSpaceBounds(const ob::StateSpacePtr& space)
{
    auto r2 = space->as<ob::RealVectorStateSpace>();
    ob::RealVectorBounds bounds(2); // 2D space
    bounds.setLow(-10.0); // Min X and Y
    bounds.setHigh(10.0); // Max X and Y
    r2->setBounds(bounds);
}

// Define a validity checker (obstacle-free space)
bool isStateValid(const ob::State* state)
{
    // All states are valid in this example
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rope_rrt_node");
    ros::NodeHandle nh;
    ROS_INFO("Rope RRT node with OMPL started!");

    // Create a 2D state space (R^2)
    auto space = std::make_shared<ob::RealVectorStateSpace>(2);
    setSpaceBounds(space);

    // Create a SpaceInformation object
    auto si = std::make_shared<ob::SpaceInformation>(space);
    si->setStateValidityChecker(isStateValid);

    // Create start and goal states
    ob::ScopedState<> start(space);
    start[0] = -5.0; // Start at (-5, -5)
    start[1] = -5.0;

    ob::ScopedState<> goal(space);
    goal[0] = 5.0; // Goal at (5, 5)
    goal[1] = 5.0;

    // Create a problem definition
    auto pdef = std::make_shared<ob::ProblemDefinition>(si);
    pdef->setStartAndGoalStates(start, goal);

    // Create the RRT planner
    auto planner = std::make_shared<og::RRT>(si);
    planner->setProblemDefinition(pdef);
    planner->setup();

    // Solve the problem with a timed termination condition (1 second)
    auto termination_condition = ob::timedPlannerTerminationCondition(1.0);
    if (planner->solve(termination_condition))
    {
        auto path = pdef->getSolutionPath()->as<og::PathGeometric>();
        ROS_INFO("Path found!");
        path->print(std::cout); // Print the solution path
    }
    else
    {
        ROS_WARN("No solution found.");
    }

    return 0;
}
