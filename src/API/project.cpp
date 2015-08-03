//
// C++ Implementation: workspace
//
// Description: 
//
//
// Author: Florian Pilardeau,B90,6349 <fpilarde@jolimont>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution

#include "project.hpp"
#include "scene.hpp"

#include "planEnvironment.hpp"
#include "hri_costspace/gestures/HRICS_gest_parameters.hpp"
#include "hri_costspace/HRICS_parameters.hpp"
//#include "SaveContext.hpp"

using namespace Move3D;
using namespace std;

Project* Move3D::global_Project = NULL;


Project::Project(Scene* sc)
{
    m_Scenes.push_back( sc );
    setActiveScene( m_Scenes.back()->getName() );

    cout << "Create project and environement parameters" << endl;
}

Project::~Project()
{
}

Move3D::Scene* Project::getActiveScene()
{
    for(unsigned int i = 0; i < m_Scenes.size(); i++)
    {
        if( m_Scenes[i]->getName().compare( m_activeScene ) == 0 )
        {
            return m_Scenes[i];
        }
    }

    return 0x00;
}

void Project::setActiveScene(string name)
{	
    m_activeScene = name;
}

void Project::insertScene(Scene* E)
{
    m_Scenes.push_back(E);
}

