/*
 *  drawModule.cpp
 *  OOMove3D
 *
 *  Created by Jim Mainprice on 18/10/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "drawModule.hpp"
#include "drawCost.hpp"

#include "P3d-pkg.h"
#include "Graphic-pkg.h"
#include "Planner-pkg.h"

#include "../p3d/env.hpp"
#include "API/Roadmap/graph.hpp"
#include "API/Roadmap/graphConverter.hpp"
#include "API/Trajectory/trajectory.hpp"

#ifdef HRI_PLANNER
#include <libmove3d/hri/hri.h>
#endif

using namespace Graphic;
using namespace Move3D;
using namespace std;

// ****************************************************************************************************
// API FUNCTIONS
// ****************************************************************************************************

static boost::function<void( double, double, double, double, double*, Move3D::Robot* )> Move3DDrawSphere;
static boost::function<void( double ,double ,double, double, double, double, int, double*, Move3D::Robot* )> Move3DDrawOneLine;
static boost::function<void( Move3D::Robot* )> Move3DDrawClearHandles;

// ****************************************************************************************************
// SETTERS
// ****************************************************************************************************

void move3d_set_fct_draw_sphere( boost::function<void( double, double, double, double, double*, Move3D::Robot* )> fct ) {  Move3DDrawSphere = fct; }
void move3d_set_fct_draw_one_line( boost::function<void( double ,double ,double, double, double, double, int, double*, Move3D::Robot*)> fct ) { Move3DDrawOneLine = fct; }
void move3d_set_fct_draw_clear_handles( boost::function<void( Move3D::Robot* )> fct ) { Move3DDrawClearHandles = fct; }

// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------

void move3d_draw_sphere( double x, double y, double z, double radius, double* color_vect,  Move3D::Robot* R )
{
    Move3DDrawSphere( x, y, z, radius, color_vect, R );
}

void move3d_draw_one_line( double x1, double y1, double z1, double x2, double y2, double z2, int color, double *color_vect, Move3D::Robot* R )
{
    Move3DDrawOneLine( x1, y1, z1, x2, y2, z2, color, color_vect, R );
}

void move3d_draw_clear_handles( Move3D::Robot* R )
{
    Move3DDrawClearHandles(R);
}

// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------

// Drawing module
DrawFunctions* global_DrawModule = NULL;

//! Class thats holding the drawing functions
DrawFunctions::DrawFunctions()
{

}

DrawFunctions::~DrawFunctions()
{
    cout << __PRETTY_FUNCTION__ << endl;
}

// Get all draw functions
std::vector<std::string> DrawFunctions::getAllDrawFunctions()
{
    std::vector< string > functions;
    std::map< string, boost::function<void()> >::iterator it;

    for ( it=functions_.begin() ; it != functions_.end(); it++ )
        functions.push_back((*it).first);

    return functions;
}

// Get active draw functions
std::vector<std::string> DrawFunctions::getActiveDrawFunctions()
{
    std::vector< string > functions;
    std::set< string >::iterator it;

    for ( it=active_functions_.begin() ; it != active_functions_.end(); it++ )
        functions.push_back(*it);

    return functions;
}

// Select the draw function with the given name in the map
bool DrawFunctions::enableDrawFunction( std::string name )
{
    cout << "SET DRAW_FUNCTION : " << name << " active" << endl;
    active_functions_.insert( name );
    return true;
}

// Select the draw function with the given name in the map
bool DrawFunctions::disableDrawFunction( std::string name )
{
    cout << "DISABLE DRAW_FUNCTION : " << name << endl;
    active_functions_.erase( active_functions_.find(name) );
    return true;
}

// Register a new draw function
void DrawFunctions::addDrawFunction( std::string name, boost::function<void()> f )
{
    cout << "ADD DRAW_FUNCTION : " << name << " , " << f << endl;
    functions_[name] = f;
}

// Delete a cost function
void DrawFunctions::deleteDrawFunction( std::string name )
{
    cout << "DELETE DRAW_FUNCTION : " << name << endl;
    disableDrawFunction( name );
    functions_.erase( name );
}

//! call to all functions in draw
void DrawFunctions::draw()
{
    // cout << __PRETTY_FUNCTION__ << endl;
    std::set< string >::iterator it;
    for ( it=active_functions_.begin() ; it != active_functions_.end(); it++ )
    {
        // cout << "DRAW_FUNCTION : " << *it << ", " << functions_[*it] << endl;
        functions_[*it]();
    }
}

//-----------------------------------------------------------------------
//-----------------------------------------------------------------------
//-----------------------------------------------------------------------

// These are function that are called
// from within libmove3d 

void g3d_export_cpp_graph()
{
    //std::cout << "API_activeGraph : " << API_activeGraph << std::endl;
    /**
    if (ENV.getBool(Env::drawGraph) && (!ENV.getBool(Env::use_p3d_structures)) && API_activeGraph )
    {
        try
        {
            if (API_activeGraph->isGraphChanged())
            {
        // Warning broken
                //XYZ_GRAPH = API_activeGraph->exportCppToGraphStruct();

        GraphConverter gc;
        XYZ_GRAPH = gc.convert(*API_activeGraph);
            }
        }
        catch(std::string str)
        {
            std::cout << "Exception in exporting the cpp graph" << std::endl;
            std::cout << str << std::endl;
        }
    }
   */
}

void g3d_draw_boost_graph()
{
    if (ENV.getBool(Env::drawGraph) && (!ENV.getBool(Env::use_p3d_structures)) && API_activeGraph )
    {
        try
        {
            //p3d_del_graph(XYZ_GRAPH);
            XYZ_GRAPH = NULL;

            API_activeGraph->draw();
        }
        catch(std::string str)
        {
            std::cout << "Exception in draw boost graph" << std::endl;
            std::cout << str << std::endl;
        }
    }
}

void g3d_draw_cost_features()
{
    // Draws the custom draw functions
    if( global_DrawModule != NULL )
        global_DrawModule->draw();

#ifdef HRI_COSTSPACE
    g3d_draw_costspace();
    g3d_draw_hrics(0);
#endif
    //std::cout << "Draw cost features" << std::endl;
    g3d_draw_grids();
}

void Graphic::initDrawFunctions()
{
    global_DrawModule = new Graphic::DrawFunctions();

    ext_g3d_traj_debug = draw_traj_debug;
    ext_g3d_draw_cost_features = (void (*)())(g3d_draw_cost_features);
    //ext_g3d_export_cpp_graph = (void (*)())(g3d_export_cpp_graph);
    ext_g3d_export_cpp_graph = (void (*)())(g3d_draw_boost_graph);
#ifdef HRI_PLANNER
    ext_g3d_draw_hri_features = g3d_hri_main;
#endif

    ext_compute_config_cost_along_traj = computeConfigCostOnTraj;
}
