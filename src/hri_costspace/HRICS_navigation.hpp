//
//  HRICS_navigation.hpp
//  libmove3d-motion
//
//  Created by Jim Mainprice on 23/04/12.
//  Copyright (c) 2012 LAAS-CNRS. All rights reserved.
//

#ifndef libmove3d_motion_HRICS_navigation_hpp
#define libmove3d_motion_HRICS_navigation_hpp

#include "API/Trajectory/trajectory.hpp"

#include "grid/HRICS_grid.hpp"
#include "grid/HRICS_two_d_grid.hpp"

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
    Navigation( Move3D::Robot* R );
    ~Navigation();
    
    Move3D::Robot* getRobot() { return m_robot; }
    
    Move3D::Trajectory* computeRobotTrajectory( Move3D::confPtr_t source, Move3D::confPtr_t target );
    Move3D::Trajectory* getSimplePath(std::vector<double> goal, std::vector<std::vector<double> >& path);
    
    void reset();
    void draw();
    double pathCost();
    void allow_smoothing(bool state);
    
  private:
    bool init();
    bool computeAStarIn2DGrid( Eigen::Vector2d source, Eigen::Vector2d target );
    bool solveAStar( PlanState* start, PlanState* goal) ;

    
    std::vector<double> m_envSize;
    
    Move3D::Robot* m_robot;
    Move3D::Robot* m_cyl;
    
    double m_maxRadius;
    
    PlanGrid* m_2DGrid;
    std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> >   m_2DPath;
    std::vector<Move3D::TwoDCell*> m_2DCellPath;
  };
}

#endif
