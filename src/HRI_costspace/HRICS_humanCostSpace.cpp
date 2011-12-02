//
//  HRICS_humanCostSpace.cpp
//  libmove3d-motion
//
//  Created by Jim Mainprice on 30/11/11.
//  Copyright 2011 LAAS/CNRS. All rights reserved.
//

#include "HRICS_humanCostSpace.hpp"

#include "HRICS_Distance.hpp"
#include "HRICS_Visibility.hpp"
#include "HRICS_Natural.hpp"

//#include "move3d-headless.h"

#include <sys/time.h>

HRICS::HumanCostSpace* global_humanCostSpace = NULL;

using namespace HRICS;
using namespace std;
using namespace tr1;

HumanCostSpace::HumanCostSpace()
{
  
}

HumanCostSpace::HumanCostSpace(Robot* rob, std::vector<Robot*> humans, double cellSize) 
: m_Robot(rob) , m_Humans( humans) 
{
  m_DistanceSpace = NULL;
  m_VisibilitySpace = NULL;
  m_ReachableSpace = NULL;
  
  initElementarySpaces();
  initHumanGrids(cellSize);
  
  if (rob->getName() == "PR2_ROBOT") 
  {
    if( !initPr2() )
    {
      cout << "Fail to init pr2 robot in HumanCostSpace" << endl;
      return;
    }
  }
  
  if (rob->getName() == "GREY_TAPE") 
  {
    if( !initGreyTape() )
    {
      cout << "Fail to init for visball" << endl;
    }
  }
}

HumanCostSpace::~HumanCostSpace()
{
  deleteHumanGrids();
  deleteElementarySpaces();
}

bool HumanCostSpace::initElementarySpaces()
{
  deleteElementarySpaces();
  
  if(m_Humans.empty())
  {
    cout << "Can not initialize elementary spazes (Humans is empty)" << endl;
    return false;
  }
  
  m_DistanceSpace = new Distance(m_Robot,m_Humans);
  m_VisibilitySpace = new Visibility(m_Humans[0]);
  m_ReachableSpace = NULL;
  return true;
}

void HumanCostSpace::deleteElementarySpaces()
{
  if(m_DistanceSpace)
    delete m_DistanceSpace;
  
  if(m_VisibilitySpace)
    delete m_VisibilitySpace;
  
  if(m_ReachableSpace)
    delete m_ReachableSpace;
}

bool HumanCostSpace::initHumanGrids(double cellsize)
{
  deleteHumanGrids();
  
  // Create 3 by 3 boxes around the human
  vector<double>  envsize(6);
  envsize[0] = -3; envsize[1] = 3;
  envsize[2] = -3; envsize[3] = 3;
  envsize[4] = -3; envsize[5] = 3;
  
  for ( int i=0; i<int(m_Humans.size()); i++) 
  {
    m_Grids.push_back( new AgentGrid( cellsize, envsize,
                                          m_Humans[i],m_DistanceSpace,m_VisibilitySpace,m_ReachableSpace) );
    
    m_Grids[i]->computeAllCellCost();
  }
  
  API_activeGrid = m_Grids[0];
  
  return true;
}

void HumanCostSpace::deleteHumanGrids()
{
  API_activeGrid = NULL;
  
  for ( int i=0; i<int(m_Grids.size()); i++) 
  {
    delete m_Grids[i];
  }
  m_Grids.clear();
}

bool HumanCostSpace::initGreyTape()
{
  m_CostJoints.clear();
  m_CostJoints.push_back( m_Robot->getJoint(1) );
  return true;
}

bool HumanCostSpace::initPr2()
{
  m_CostJoints.clear();
  
//  m_CostJoints.push_back( m_Robot->getJoint(1) );
  
//  for (int i=0; i<int(m_Robot->getNumberOfJoints()); i++) 
//  {
//    string name = m_Robot->getJoint(i)->getName();
//    
//    if(name == "virtual_object_left" || 
//       name == "virtual_object_right" || 
//       name == "J0" )
//      continue;
//    
//    m_CostJoints.push_back( m_Robot->getJoint(i) );
//    cout << "Add joint : " << m_CostJoints.back()->getName() << endl;
//  }
  
  for (int i=0; i<int(m_Robot->getNumberOfJoints()); i++) 
  {
    string name = m_Robot->getJoint(i)->getName();
    
    if(name == "platformJoint" || name == "Torso" || name == "pan_cam" || 
       name == "right-Arm1" || 
       name == "right-Arm2" || 
       name == "right-Arm3" || 
       name == "right-Arm4" || 
       name == "right-Arm5" || 
       name == "right-Arm6" || 
       name == "right-Arm7" )
    {
      m_CostJoints.push_back( m_Robot->getJoint(i) );
      cout << "Add joint : " << m_CostJoints.back()->getName() << endl;
    }
  }
  
  cout << m_CostJoints.size() << " nb of cost joints" << endl;
  return true;
}

void HumanCostSpace::testCostFunction()
{ 
  shared_ptr<Configuration> q;
  
  bool first_loop=true;
  unsigned int iter=0;
  double t=0.0,t_init=0.0;

  while( (t - t_init) < 10.0 )
  {    
    q = m_Robot->shoot();
    
//    if( !q->isInCollision() )
//    {
      this->getCost(*q);
//    }
    
    //g3d_draw_allwin_active();
    
    timeval tim;
    gettimeofday(&tim, NULL);
    t = tim.tv_sec+(tim.tv_usec/1000000.0);
    
    if(first_loop)
    {
      t_init = t;
      first_loop = false;
    }
    iter++;
  }
  cout << "iter : " << iter << " in " << (t - t_init) << " sec. " << endl;
  cout << "cost per sec : " << double(iter)/(t - t_init) << endl;
}

double HumanCostSpace::getCost(Configuration& q)
{
  m_Robot->setAndUpdate(q);
  
  double cost=0.0;
  
  for (int i=0; i<int(m_Grids.size()); i++) 
  {
    for (int j=0; j<int(m_CostJoints.size()); j++) 
    {
      cost += m_Grids[i]->getCellCostAt( m_CostJoints[j]->getVectorPos() );
    }
  }
  return cost;
}

