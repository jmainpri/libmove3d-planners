/*
 * Copyright (c) 2010-2014 LAAS/CNRS, WPI
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 *
 *                                               Jim Mainprice Tue 27 May 2014 
 */
#include "HRICS_parameters.hpp"
#include <libmove3d/include/move3d-headless.h>
#include <iostream>
//#include "../p3d/env.hpp"

// A new container is created for each module
// First declaire the maps of praramters
// Then fill in the maps that associate the enum to the Qt container
// When Qt is disabled this just acts as a normal container

// Definition of the parameter container
Parameters<
HricsParam::boolParameter,
HricsParam::intParameter,
HricsParam::doubleParameter,
HricsParam::stringParameter,
HricsParam::vectorParameter>* HriEnv = NULL;

#ifdef QT_LIBRARY
HricsParam* EnumHricsParameterObject = NULL;

HricsParam::HricsParam()
{

}

HricsParam::~HricsParam()
{

}
#endif

// @brief Function that inizializes the
// Parameter container
void initHricsParameters()
{
#ifdef QT_LIBRARY
    EnumHricsParameterObject = new HricsParam;
#endif

    // Create 5 maps for all types and fill the 5 maps
    // ------------------------------------------------------------------
    std::map<HricsParam::boolParameter,      boolContainer*>                  myBoolMap;
    std::map<HricsParam::intParameter,       intContainer*>                   myIntMap;
    std::map<HricsParam::doubleParameter,    doubleContainer*>                myDoubleMap;
    std::map<HricsParam::stringParameter,    stringContainer*>                myStringMap;
    std::map<HricsParam::vectorParameter,    vectorContainer*>                myVectorMap;

    // Bool
    // ------------------------------------------------------------------
    myBoolMap.insert( std::make_pair( HricsParam::init_spheres_cost, new boolContainer(false) ));
    myBoolMap.insert( std::make_pair( HricsParam::init_human_trajectory_cost, new boolContainer(false) ));
    myBoolMap.insert( std::make_pair( HricsParam::ioc_single_iteration, new boolContainer(false) ));
    myBoolMap.insert( std::make_pair( HricsParam::ioc_load_samples_from_file, new boolContainer(false) ));
    myBoolMap.insert( std::make_pair( HricsParam::ioc_draw_demonstrations, new boolContainer(false) ));
    myBoolMap.insert( std::make_pair( HricsParam::ioc_draw_samples, new boolContainer(false) ));
    myBoolMap.insert( std::make_pair( HricsParam::ioc_sample_around_demo, new boolContainer(false) ));
    myBoolMap.insert( std::make_pair( HricsParam::ioc_exit_after_run, new boolContainer(false) ));
    myBoolMap.insert( std::make_pair( HricsParam::ioc_use_stomp_spetial_cost, new boolContainer(false) ));
    myBoolMap.insert( std::make_pair( HricsParam::ioc_use_simulation_demos, new boolContainer(false) ));
    myBoolMap.insert( std::make_pair( HricsParam::ioc_user_set_pelvis_bounds, new boolContainer(false) ));
    myBoolMap.insert( std::make_pair( HricsParam::ioc_use_baseline, new boolContainer(false) ));
    myBoolMap.insert( std::make_pair( HricsParam::ioc_no_replanning, new boolContainer(false) ));
    myBoolMap.insert( std::make_pair( HricsParam::ioc_split_motions, new boolContainer(false) ));

    // Int
    // ------------------------------------------------------------------
    myIntMap.insert( std::make_pair( HricsParam::ioc_phase, new intContainer(0) ));
    myIntMap.insert( std::make_pair( HricsParam::ioc_sample_iteration, new intContainer(0) ));
    myIntMap.insert( std::make_pair( HricsParam::ioc_nb_of_way_points, new intContainer(0) ));
    myIntMap.insert( std::make_pair( HricsParam::ioc_planner_type, new intContainer(0) ));
    myIntMap.insert( std::make_pair( HricsParam::ioc_spheres_to_draw, new intContainer(-1) ));
    myIntMap.insert( std::make_pair( HricsParam::ioc_from_file_offset, new intContainer(-1) ));
    myIntMap.insert( std::make_pair( HricsParam::ioc_ik, new intContainer(-1) ));
    myIntMap.insert( std::make_pair( HricsParam::ioc_baseline_type, new intContainer(-1) ));

    // Double
    // ------------------------------------------------------------------
    myDoubleMap.insert( std::make_pair( HricsParam::ioc_spheres_power, new doubleContainer(2.0) ));
    myDoubleMap.insert( std::make_pair( HricsParam::ioc_sample_std_dev, new doubleContainer(2.0) ));
    myDoubleMap.insert( std::make_pair( HricsParam::ioc_sample_std_dev_ik, new doubleContainer(2.0) ));
    myDoubleMap.insert( std::make_pair( HricsParam::ioc_cost_factor, new doubleContainer(2.0) ));

    //cout << "PlanEnv->getDouble(p) = " << PlanEnv->getDouble( PlanParam::env_objectNessecity ) << endl;

    // String
    // ------------------------------------------------------------------
#ifdef QT_LIBRARY
    myStringMap.insert(std::make_pair(HricsParam::ioc_traj_split_name,   new stringContainer("titi")));

#endif

    // Vector
    // ------------------------------------------------------------------
    std::vector<double> tutu;
    tutu.push_back( 1 ); tutu.push_back( 8 );

    myVectorMap.insert(std::make_pair(HricsParam::tutu,                       new vectorContainer(tutu)));

    // Make the new parameter container
    HriEnv =  new Parameters<
            HricsParam::boolParameter,
            HricsParam::intParameter,
            HricsParam::doubleParameter,
            HricsParam::stringParameter,
            HricsParam::vectorParameter>(
                myBoolMap,
                myIntMap,
                myDoubleMap,
                myStringMap,
                myVectorMap);
}

