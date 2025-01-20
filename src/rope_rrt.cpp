#include <ros/ros.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

int main(int argc, char **argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "rope_shortcut_path_example");
    ros::NodeHandle nh;

    // Define a 2D state space
    auto space(std::make_shared<ompl::base::RealVectorStateSpace>(2));
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(-10);
    bounds.setHigh(10);
    space->setBounds(bounds);

    // Create space information
    auto si = std::make_shared<ompl::base::SpaceInformation>(space);

    // Define a geometric path in the space
    ompl::geometric::PathGeometric path(si);
    for (int i = 0; i < 5; ++i)
    {
        auto state = si->allocState()->as<ompl::base::RealVectorStateSpace::StateType>();
        state->values[0] = i * 2.0; // x-coordinate
        state->values[1] = i * 2.0; // y-coordinate
        path.append(state);
    }

    // Print original path
    ROS_INFO("Original Path:");
    for (size_t i = 0; i < path.getStateCount(); ++i)
    {
        const auto *state = path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
        ROS_INFO("State %lu: [%f, %f]", i, state->values[0], state->values[1]);
    }

    // Simplify the path using ropeShortcutPath
    ompl::geometric::PathSimplifier simplifier(si);
    double delta = 1.0;                  // Step size
    double equivalenceTolerance = 0.1;  // Equivalence tolerance
    bool improved = simplifier.ropeShortcutPath(path, delta, equivalenceTolerance);

    if (improved)
    {
        ROS_INFO("Path improved with ropeShortcutPath.");
    }
    else
    {
        ROS_INFO("No improvements made to the path.");
    }

    // Print simplified path
    ROS_INFO("Simplified Path:");
    for (size_t i = 0; i < path.getStateCount(); ++i)
    {
        const auto *state = path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
        ROS_INFO("State %lu: [%f, %f]", i, state->values[0], state->values[1]);
    }

    return 0;
}
