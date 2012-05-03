//
//  HRICS_Navigation.cpp
//  libmove3d-motion
//
//  Created by Jim Mainprice on 23/04/12.
//  Copyright (c) 2012 LAAS-CNRS. All rights reserved.
//

#include "HRICS_Navigation.hpp"

#include "API/ConfigSpace/configuration.hpp"
#include "Grid/HRICS_Grid.hpp"
#include "planner/TrajectoryOptim/Classic/smoothing.hpp"

#include "Graphic-pkg.h"

using namespace std;
using namespace tr1;
using namespace Eigen;
using namespace HRICS;

/**
 * Take as input a robot and a human and performs init
 */
Navigation::Navigation(Robot* R) : m_robot(R)
{
  init();
}

Navigation::~Navigation()
{
  delete m_2DGrid;
}

/**
 * Initializes the local variables
 */
bool Navigation::init()
{
  m_envSize = global_Project->getActiveScene()->getBounds();
  m_envSize.resize(4);
  
  double diagonal = std::sqrt( std::pow(m_envSize[1]-m_envSize[0], 2 ) + std::pow(m_envSize[3]-m_envSize[2] , 2 ) );
  cout << "Diagonal/50 : " << diagonal/50 << endl;
  
  m_2DGrid = new PlanGrid( m_robot,/*ENV.getDouble(Env::PlanCellSize)*/ diagonal/50, m_envSize );
  API_activeGrid = m_2DGrid;
  
  return true;
}

void Navigation::reset()
{
  m_2DGrid->reset();
}

/**
 * Computes a trajectory
 */
API::Trajectory* Navigation::computeRobotTrajectory( confPtr_t source, confPtr_t target )
{
  Vector2d x1,x2;
  
  x1[0]=(*source)[6];
  x1[1]=(*source)[7]; 
  
  x2[0]=(*target)[6];
  x2[1]=(*target)[7];
  
  m_robot->setAndUpdate(*source);
  
  if( computeAStarIn2DGrid( x1, x2 ) )
  {
    API::Trajectory* traj = new API::Trajectory(m_robot);
    
    for(int i=0;i<int(m_2DPath.size());i++)
    {
      confPtr_t q = m_robot->getCurrentPos();
      (*q)[6] = m_2DPath[i][0];
      (*q)[7] = m_2DPath[i][1];
      
      traj->push_back( q );
    }
    traj->replaceP3dTraj();
    return traj;
  }
  else
  {
    return NULL;
  }
}

/**
 * Takes the robot initial config and calls the solve A*
 * to compute the 2D path
 */
bool Navigation::computeAStarIn2DGrid( Vector2d source, Vector2d target )
{
  PlanCell* startCell = dynamic_cast<PlanCell*>(m_2DGrid->getCell(source));
  Vector2i startCoord = startCell->getCoord();
  cout << "Start Pos = (" << source[0] << " , " << source[1] << ")" << endl;
  cout << "Start Coord = (" << startCoord[0] << " , " << startCoord[1] << ")" << endl;
  
  PlanState* start = new PlanState( startCell, m_2DGrid );
  
  PlanCell* goalCell = dynamic_cast<PlanCell*>(m_2DGrid->getCell(target));
  Vector2i goalCoord = goalCell->getCoord();
  cout << "Goal Pos = (" << target[0] << " , " << target[1] << ")" << endl;
  cout << "Goal Coord = (" << goalCoord[0] << " , " << goalCoord[1] << ")" << endl;
  
  if( startCoord == goalCoord )
  {
    cout << " no planning as cells are identical" << endl;
    return false;
  }
  
  PlanState* goal = new PlanState( goalCell, m_2DGrid );
  
  if ( start == NULL || goal == NULL ) {
    cout << "Start or goal == NULL" << endl;
    return false;
  }
  
  if( solveAStar( start, goal ) ) 
  {
    double SumOfCost= 0.0;
    for(int i=0; i< int(m_2DPath.size()); i++ )
    {
      //cout << "Cell "<< i <<" = " << endl << m_2DPath[i] << endl;
      SumOfCost +=  dynamic_cast<PlanCell*>(m_2DCellPath[i])->getCost();
    }
    cout << " SumOfCost = "  << SumOfCost << endl;
    return true;
  }
  else { 
    return false; 
  }
}

/**
 * Solve A Star in a 2D grid using the API A Star on
 * takes as input A* States
 */
bool Navigation::solveAStar( PlanState* start, PlanState* goal )
{
  bool path_exists=true;
  m_2DPath.clear();
  
  // Change the way AStar is computed to go down
  if( start->getCell()->getCost() < goal->getCell()->getCost() )
  {
    API::AStar* search = new API::AStar(start);
    vector<API::State*> path = search->solve(goal);
    
    if(path.size() == 0 )
    {
      m_2DPath.clear();
      m_2DCellPath.clear();
      path_exists = false;
      return path_exists;
    }
    
    for (unsigned int i=0;i<path.size();i++)
    {
      API::TwoDCell* cell = dynamic_cast<PlanState*>(path[i])->getCell();
      m_2DPath.push_back( cell->getCenter() );
      m_2DCellPath.push_back( cell );
    }
  }
  else
  {
    API::AStar* search = new API::AStar(goal);
    vector<API::State*> path = search->solve(start);
    
    if(path.size() == 0 )
    {
      m_2DPath.clear();
      m_2DCellPath.clear();
      path_exists = false;
      return path_exists;
    }
    
    for (int i=path.size()-1;i>=0;i--)
    {
      PlanCell* cell = dynamic_cast<PlanState*>(path[i])->getCell();
      m_2DPath.push_back( cell->getCenter() );
      m_2DCellPath.push_back( cell );
    }
  }
  
  return path_exists;
}

/**
 * Draws the 3D path as a yellow line
 */
void Navigation::draw()
{
  for(int i=0;i<int(m_2DPath.size())-1;i++)
  {
    glLineWidth(3.);
    g3d_drawOneLine(m_2DPath[i][0],      m_2DPath[i][1],      0.4,
                    m_2DPath[i+1][0],    m_2DPath[i+1][1],    0.4,
                    Yellow, NULL);
    glLineWidth(1.);
  }
}
