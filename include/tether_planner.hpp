#ifndef TETHER_PLANNER_HPP
#define TETHER_PLANNER_HPP

#include "global_vars.hpp"
#include "helper_functions.hpp"
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <Eigen/Dense>
#include <memory>
#include <vector>


//implement tether planner class

// input 

//global goal point
//current position of the robot
// L_max max tether length
// 



class TetherPlanner
{

    public:
        TetherPlanner(); // Constructor declaration

        

        void findNextGoal(const ompl::geometric::PathGeometric &tether , 
                const std::vector<double> &current_position, 
                const std::vector<double> &goal, 
                double &L_max,         
                const std::shared_ptr<ompl::base::RealVectorStateSpace> &space,
                const std::shared_ptr<ompl::base::SpaceInformation> &si);


      ompl::geometric::PathGeometric InvertTetherPath(const ompl::geometric::PathGeometric &tether
                                     , const std::shared_ptr<ompl::base::SpaceInformation> &si);
      double findTetherLength(const ompl::geometric::PathGeometric &path);
      
      bool isEqual(const std::vector<double> &point1, const std::vector<double> &point2) const;

};



#endif