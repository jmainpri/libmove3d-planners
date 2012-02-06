//
// C++ Implementation: environnement
//
// Description: 
//
//
// Author: Florian Pilardeau,B90,6349 <fpilarde@jolimont>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution

#include "scene.hpp"

#include "P3d-pkg.h"
#include "Planner-pkg.h"

#include "planner/plannerFunctions.hpp"

using namespace std;

std::string global_ActiveRobotName;

Scene::Scene(p3d_env* environnement)
{
	m_Scene = environnement;
	
        m_Name = m_Scene->name;
	
	for (int i=0; i<m_Scene->nr; i++) 
	{
		cout << "Add new Robot to Scene : " << m_Scene->robot[i]->name << endl;
		m_Robot.push_back( new Robot( m_Scene->robot[i] ) );
		m_Robot.back()->setActiveScene( this );
	}
  
  // For all HRI planner the robot 
  // Will be set here
  if (m_Scene->active_robot) 
  {
    global_ActiveRobotName = m_Scene->active_robot->name;
    cout << "The Scene global_ActiveRobotName is : " << global_ActiveRobotName << endl;
  }
  else 
  {
    global_ActiveRobotName = "";
    cout << "The Scene global_ActiveRobotName has not been set" << endl;
  }
  
	// Set the robot by name containing ROBOT to active
	Robot* rob = getRobotByNameContaining(global_ActiveRobotName);
	
	if (rob != NULL) 
	{
		setActiveRobot(rob->getName());
	}
	
	// Set the active rrt function
	ext_p3d_run_rrt = p3d_run_rrt;
}


Scene::~Scene()
{
}

string Scene::getName()
{
  return m_Name;
}

void Scene::setActiveRobot(string name)
{
	unsigned int id = getRobotId(name);
	p3d_sel_desc_id(P3D_ROBOT,m_Scene->robot[id]);
}

Robot* Scene::getActiveRobot()
{
	return getRobotByName(m_Scene->cur_robot->name);
}

unsigned int Scene::getRobotId(string str)
{
	for (unsigned int i=0; i<m_Robot.size(); i++) 
	{
		if ( m_Robot[i]->getName().compare( str ) == 0 ) 
		{
			return i;
		}
	}
	
	cout << "Error geting robot id in " << __func__ << endl;
	return 0;
}

/**
 * Get robot by name
 */
Robot* Scene::getRobotByName(string str)
{
	unsigned int id = getRobotId(str);
	
	if (id < m_Robot.size() ) 
	{
		return m_Robot[id];
	}

	return NULL;
}

/**
 * Get robot by name containing
 */
Robot* Scene::getRobotByNameContaining(string str)
{
	for (unsigned int i=0; i<m_Robot.size(); i++) 
	{
		if ( m_Robot[i]->getName().find( str ) != string::npos ) 
		{
			return m_Robot[i];
		}
	}

	return NULL;
}

void Scene::insertRobot(Robot* R)
{
  m_Robot.push_back(R);
}

double Scene::getDMax()
{
	return m_Scene->dmax;
}

std::vector<double> Scene::getBounds()
{
  vector<double> bounds(6);
  bounds[0] = m_Scene->box.x1;
  bounds[1] = m_Scene->box.x2;
  bounds[2] = m_Scene->box.y1;
  bounds[3] = m_Scene->box.y2;
  bounds[4] = m_Scene->box.z1;
  bounds[5] = m_Scene->box.z2;
  
  return bounds;
}

