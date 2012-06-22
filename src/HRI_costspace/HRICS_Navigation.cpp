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
#include "planner/TrajectoryOptim/plannarTrajectorySmoothing.hpp"
#include "planEnvironment.hpp"

#include "Graphic-pkg.h"
#include "P3d-pkg.h"
#include "Collision-pkg.h"

std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> >   path_to_draw;

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
  p3d_col_activate_rob_rob(m_robot->getRobotStruct(),m_cyl->getRobotStruct());
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
  double pace = 0.20;
  cout << "pace : " << pace << " meters" << endl;

  for (int i=0; i<XYZ_ENV->nr; i++)
  {
      string name(XYZ_ENV->robot[i]->name);
      if (m_robot->getName().find("ROBOT") != string::npos)
      {
          if(name.find("PR_2CYLINDER") != string::npos )
          {
              m_cyl = new Robot(XYZ_ENV->robot[i]);
              break;
          }
      }
  }
  if (m_cyl == NULL)
  {
      cout<< "ERROR: no cylinder found" <<endl;
      return false;
  }
  p3d_col_deactivate_rob_rob(m_robot->getRobotStruct(),m_cyl->getRobotStruct());
  m_2DGrid = new PlanGrid( m_cyl,/*ENV.getDouble(Env::PlanCellSize)*/ pace, m_envSize );
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
  confPtr_t q = m_cyl->getCurrentPos();
  (*q)[6] =0;
  (*q)[7] =0;

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
    m_cyl->setAndUpdate(*q);
    return traj;
  }
  else
  {
    m_cyl->setAndUpdate(*q);
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

bool Navigation::getSimplePath(std::vector<double> goal, std::vector<std::vector<double> >& path)
{
    bool c_tmp = ENV.getBool(Env::isCostSpace);
    ENV.setBool(Env::isCostSpace,true);
    confPtr_t i = m_robot->getCurrentPos();
//    confPtr_t q = m_robot->getCurrentPos();
    confPtr_t g = m_robot->getCurrentPos();
    int firstIndexOfRobotDof = dynamic_cast<p3d_jnt*>(m_robot->getRobotStruct()->baseJnt)->user_dof_equiv_nbr;
    (*g)[firstIndexOfRobotDof + 0] = goal[0];
    (*g)[firstIndexOfRobotDof + 1] = goal[1];
    (*g)[firstIndexOfRobotDof + 5] = goal[2];

    if (!computeRobotTrajectory(i,g))
    {
        m_robot->setAndUpdate(*i);
        ENV.setBool(Env::isCostSpace,c_tmp);
        return false;
    }
    std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > robotTraj3D;
    PlannarTrajectorySmoothing PTS(m_robot);
    Eigen::Vector2d b;
    b[0] = goal[0];
    b[1] = goal[1];
    m_2DPath.push_back(b);
    bool tmp = PlanEnv->getBool(PlanParam::env_createTrajs);
    if (tmp)
    {
       m_2DPath = PTS.smoothTrajectory(m_robot,m_2DPath);
    }
    robotTraj3D = PTS.add3DimwithoutTrajChange(m_2DPath,m_robot,0.05);

    Eigen::Vector3d v;
    v[0] = goal[0];
    v[1] = goal[1];
    v[2] = goal[2];

    robotTraj3D.push_back(v);
    API::Trajectory t(m_robot);


    for (unsigned int j = 0; j < robotTraj3D.size();j++)
    {
        std::vector<double> p;
        p.push_back(robotTraj3D.at(j)[0]);
        p.push_back(robotTraj3D.at(j)[1]);
        p.push_back(robotTraj3D.at(j)[2]);
        path.push_back(p);

        confPtr_t q = m_robot->getInitialPosition();
        (*q)[firstIndexOfRobotDof + 0] = p[0];
        (*q)[firstIndexOfRobotDof + 1] = p[1];
        (*q)[firstIndexOfRobotDof + 5] = p[2];
        t.push_back(q);
    }

    t.replaceP3dTraj();

    path_to_draw = PTS.get2DtrajFrom3Dtraj(robotTraj3D);

    m_robot->setAndUpdate(*i);
    ENV.setBool(Env::isCostSpace,c_tmp);
    return true;
}

void Navigation::allow_smoothing(bool state)
{
    PlanEnv->setBool(PlanParam::env_createTrajs,state);
}
