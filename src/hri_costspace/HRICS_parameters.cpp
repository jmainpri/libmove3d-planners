#include "HRICS_parameters.hpp"
#include "move3d-headless.h"
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
    myBoolMap.insert( std::make_pair( HricsParam::ioc_single_iteration, new boolContainer(false) ));
    myBoolMap.insert( std::make_pair( HricsParam::ioc_load_samples_from_file, new boolContainer(false) ));
    myBoolMap.insert( std::make_pair( HricsParam::ioc_draw_demonstrations, new boolContainer(false) ));
    myBoolMap.insert( std::make_pair( HricsParam::ioc_draw_samples, new boolContainer(false) ));
    myBoolMap.insert( std::make_pair( HricsParam::ioc_sample_around_demo, new boolContainer(false) ));
    myBoolMap.insert( std::make_pair( HricsParam::ioc_exit_after_run, new boolContainer(false) ));
    myBoolMap.insert( std::make_pair( HricsParam::ioc_use_stomp_spetial_cost, new boolContainer(false) ));

    // Int
    // ------------------------------------------------------------------
    myIntMap.insert( std::make_pair( HricsParam::ioc_phase, new intContainer(0) ));
    myIntMap.insert( std::make_pair( HricsParam::ioc_sample_iteration, new intContainer(0) ));
    myIntMap.insert( std::make_pair( HricsParam::ioc_nb_of_way_points, new intContainer(0) ));
    myIntMap.insert( std::make_pair( HricsParam::ioc_planner_type, new intContainer(0) ));
    myIntMap.insert( std::make_pair( HricsParam::ioc_spheres_to_draw, new intContainer(-1) ));
    myIntMap.insert( std::make_pair( HricsParam::ioc_from_file_offset, new intContainer(-1) ));

    // Double
    // ------------------------------------------------------------------
    myDoubleMap.insert( std::make_pair( HricsParam::ioc_spheres_power, new doubleContainer(2.0) ));
    myDoubleMap.insert( std::make_pair( HricsParam::ioc_sample_std_dev, new doubleContainer(2.0) ));
    myDoubleMap.insert( std::make_pair( HricsParam::ioc_cost_factor, new doubleContainer(2.0) ));

    //cout << "PlanEnv->getDouble(p) = " << PlanEnv->getDouble( PlanParam::env_objectNessecity ) << endl;

    // String
    // ------------------------------------------------------------------
#ifdef QT_LIBRARY
    myStringMap.insert(std::make_pair(HricsParam::titi,                       new stringContainer("titi")));
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

