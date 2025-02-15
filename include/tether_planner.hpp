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

        std::vector<std::vector<double>> exit_points_list_; // list of potential exit points for replanning


        double delta_ = 0.1;                // Step size
        double equivalenceTolerance_ = 0.000001;  // Equivalence tolerance
        double Alternative_Path_Tether_Length = 100000;

        double Direct_Path_Tether_Length = 100000;

        ompl::geometric::PathGeometric findNextGoal(const ompl::geometric::PathGeometric &tether , 
                                        const std::vector<double> &current_position, 
                                        const std::vector<double> &goal, 
                                        double &L_max,         
                                        const std::shared_ptr<ompl::base::RealVectorStateSpace> &space,
                                        const std::shared_ptr<ompl::base::SpaceInformation> &si);
       

       ompl::geometric::PathGeometric ReplanPath(const ompl::geometric::PathGeometric &tether , 
                                                const std::vector<double> &current_position, 
                                                const std::vector<double> &goal, 
                                                double &L_max,         
                                                const std::shared_ptr<ompl::base::RealVectorStateSpace> &space,
                                                const std::shared_ptr<ompl::base::SpaceInformation> &si);

      
      ompl::geometric::PathGeometric DirectPath( const std::vector<double> &current_position, 
                                                const std::vector<double> &goal, 
                                                const std::shared_ptr<ompl::base::RealVectorStateSpace> &space,
                                                const std::shared_ptr<ompl::base::SpaceInformation> &si);
       

      ompl::geometric::PathGeometric InvertTetherPath(const ompl::geometric::PathGeometric &tether
                                     , const std::shared_ptr<ompl::base::SpaceInformation> &si);
      double findTetherLength(const ompl::geometric::PathGeometric &path);
      
      bool isEqual(const std::vector<double> &point1, const std::vector<double> &point2) const;
      
      void UpdateExitPointsList(const ompl::geometric::PathGeometric &tether);

        ompl::geometric::PathGeometric FindShortestPath(const ompl::base::ScopedState<ompl::base::RealVectorStateSpace> &start,
                                                        const ompl::base::ScopedState<ompl::base::RealVectorStateSpace> &goal,
                                                        const std::shared_ptr<ompl::base::SpaceInformation> &si);

     double findTetherLengthReplannedPath(const ompl::geometric::PathGeometric &tether, 
                                               const std::vector<double> &exit_point, 
                                                const std::vector<double> &goal,
                                                const std::shared_ptr<ompl::base::RealVectorStateSpace> &space,
                                                 const std::shared_ptr<ompl::base::SpaceInformation> &si);


     std::vector<double> FindNextPointAlongPath(const std::vector<double> &current_position, 
                                                          const std::vector<double> &goal_last, 
                                                          const ompl::geometric::PathGeometric &path_replan);



ompl::geometric::PathGeometric ReplanPath(const ompl::geometric::PathGeometric &tether, 
                                                         const std::vector<double> &exit_point, 
                                                         const std::vector<double> &goal,
                                                         const std::shared_ptr<ompl::base::SpaceInformation> &si);



ompl::geometric::PathGeometric CalculateAlternativePath_i(const int node_n, 
                                                                  ompl::geometric::PathGeometric tether,
                                                                  const std::vector<double> goal,
                                                                  const std::shared_ptr<ompl::base::SpaceInformation> &si);
   


ompl::geometric::PathGeometric  SearchAlternativePath( ompl::geometric::PathGeometric tether,
                                            const std::vector<double> goal,
                                            const std::shared_ptr<ompl::base::SpaceInformation> &si,
                                            const double L_max);
                              
                              
std::vector<double> getNextGoal(const ompl::geometric::PathGeometric &path);

ompl::geometric::PathGeometric computePathSegment2(const ompl::geometric::PathGeometric &Path_segment1,
  const std::vector<double> &goal,
  const std::shared_ptr<ompl::base::SpaceInformation> &si);


};



#endif