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
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

// Include nvblox headers
//#include <nvblox/core/volume.h> // Adjust the path based on your nvblox installation
//#include <nvblox/core/utils.h>   // Include other relevant nvblox headers as needed




//implement tether planner class

// input 

//global goal point
//current position of the robot
// L_max max tether length
// 



class TetherPlanner
{
    
    public:
       // TetherPlanner( ); // Constructor declaration
        TetherPlanner(double delta, double equivalenceTolerance); // Constructor declaration

        std::vector<std::vector<double>> exit_points_list_; // list of potential exit points for replanning


        double delta_;                // Step size
        double equivalenceTolerance_;  // Equivalence tolerance
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


  std::vector<double> GetNextPointAlongPath( const ompl::geometric::PathGeometric &path,  
                                                       const std::vector<double> current_position,
                                             const std::vector<double> goal,
                                             const std::shared_ptr<ompl::base::SpaceInformation> &si );




std::vector<double> MoveGoalToSafeZone(const std::vector<double> &node_n1, 
const std::vector<double> &node_n2, 
const std::vector<double> &node_n3, 
double delta_safe, const std::shared_ptr<ompl::base::SpaceInformation> &si);
 
  
  
std::vector<double> computePerpendicularUnitVector(const std::vector<double> &v1, const std::vector<double> &v2);

  

std::vector<double> SearchRandomDirection();
  
  
ompl::geometric::PathGeometric OffsetPath(const ompl::geometric::PathGeometric &path, 
           const std::shared_ptr<ompl::base::SpaceInformation> &si, double delta_safe);
  
                                     
          
};








#endif