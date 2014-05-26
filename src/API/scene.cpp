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

#include "Planner-pkg.h"

#include "planner/plannerFunctions.hpp"

using namespace Move3D;
using namespace std;

std::string global_ActiveRobotName;

// ****************************************************************************************************
// API FUNCTIONS
// ****************************************************************************************************

static boost::function<void*(void*, std::string&, std::vector<Robot*>&, std::string& )> Move3DSceneConstructor;
static boost::function<void( Scene*, void*, const std::string& )> Move3DSceneSetActiveRobot;
static boost::function<Robot*( Scene*, void* )> Move3DSceneGetActiveRobot;
static boost::function<double( void* )> Move3DSceneGetDMax;
static boost::function<std::vector<double>( void* )> Move3DSceneGetBounds;

// ****************************************************************************************************
// SETTERS
// ****************************************************************************************************

void move3d_set_fct_scene_constructor( boost::function<void*(void*, std::string&, std::vector<Robot*>&, std::string& )> fct ) {  Move3DSceneConstructor = fct; }
void move3d_set_fct_set_active_robot( boost::function<void( Scene*, void*, const std::string& )> fct ) {  Move3DSceneSetActiveRobot = fct; }
void move3d_set_fct_get_active_robot( boost::function<Robot*( Scene*, void* )> fct ) {  Move3DSceneGetActiveRobot = fct; }
void move3d_set_fct_get_dmax( boost::function<double(void*)> fct ) {  Move3DSceneGetDMax = fct; }
void move3d_set_fct_get_bounds( boost::function<std::vector<double>( void* )> fct ) {  Move3DSceneGetBounds = fct; }

// ****************************************************************************************************
// CLASS
// ****************************************************************************************************

Move3D::Scene::Scene( void* environnement )
{
    m_Scene = Move3DSceneConstructor( environnement, m_Name, m_Robot, global_ActiveRobotName );

    // Set the robot by name containing ROBOT to active
    Robot* rob = getRobotByNameContaining( global_ActiveRobotName );

    if( rob != NULL )
    {
        setActiveRobot( rob->getName() );
    }

    for( size_t i=0;i<m_Robot.size(); i++ )
    {
        m_Robot[i]->setActiveScene( this );
    }

    // Set the active rrt function
    ext_p3d_run_rrt = p3d_run_rrt;
}

Move3D::Scene::~Scene()
{
}

string Scene::getName()
{
    return m_Name;
}

void Scene::setActiveRobot( const string &name )
{
    Move3DSceneSetActiveRobot( this, m_Scene, name );
}

Robot* Scene::getActiveRobot()
{
    return Move3DSceneGetActiveRobot( this, m_Scene );
}

int Scene::getRobotId(const string& str)
{
    for( size_t i=0; i<m_Robot.size(); i++ )
    {
        if ( m_Robot[i]->getName() == str )
        {
            return i;
        }
    }

    cout << "Error getting robot id in " << __PRETTY_FUNCTION__ << endl;
    return -1;
}

/**
 * Get robot by name
 */
Robot* Scene::getRobotByName(const string &str)
{
    int id = getRobotId(str);

    if ( id >= 0 && id < int(m_Robot.size()) )
    {
        return m_Robot[id];
    }

    return NULL;
}

/**
 * Get robot by name containing a sub string
 */
Robot* Scene::getRobotByNameContaining(const string &str)
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
    return Move3DSceneGetDMax( m_Scene );
}

std::vector<double> Scene::getBounds()
{
    return Move3DSceneGetBounds( m_Scene );
}

