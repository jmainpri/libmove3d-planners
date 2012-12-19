//
//  HRICS_Navigation.hpp
//  libmove3d-motion
//
//  Created by Jim Mainprice on 23/04/12.
//  Copyright (c) 2012 LAAS-CNRS. All rights reserved.
//

#ifndef libmove3d_motion_HRICS_Navigation_hpp
#define libmove3d_motion_HRICS_Navigation_hpp

#include "API/Trajectory/trajectory.hpp"

#include "Grid/HRICS_Grid.hpp"
#include "Grid/HRICS_TwoDGrid.hpp"

#include <Eigen/StdVector>



/**
 @defgroup HRICS Hri Cost space
 @ingroup HRICS
 */
namespace HRICS
{
  class Navigation 
  {
  public:
    Navigation(Robot* R);
    ~Navigation();
    
    Robot* getRobot() { return m_robot; }
    
    API::Trajectory* computeRobotTrajectory( confPtr_t source, confPtr_t target );
    API::Trajectory* getSimplePath(std::vector<double> goal, std::vector<std::vector<double> >& path);
    
    void reset();
    void draw();
    double pathCost();
    void allow_smoothing(bool state);
    
  private:
    bool init();
    bool computeAStarIn2DGrid( Eigen::Vector2d source, Eigen::Vector2d target );
    bool solveAStar(PlanState* start,PlanState* goal);

    
    std::vector<double> m_envSize;
    
    Robot* m_robot;
    Robot* m_cyl;
    
    double m_maxRadius;
    
    PlanGrid* m_2DGrid;
    std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> >   m_2DPath;
    std::vector<API::TwoDCell*> m_2DCellPath;
  };
}

#endif
