/*
 *  PlanEnvironment.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 11/10/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "HRICS_GestParameters.hpp"
#include "move3d-headless.h"
#include <iostream>
//#include "../p3d/env.hpp"

// A new container is created for each module
// First declaire the maps of praramters
// Then fill in the maps that associate the enum to the Qt container
// When Qt is disabled this just acts as a normal container

// Definition of the parameter container
Parameters<
GestParam::boolParameter,
GestParam::intParameter,
GestParam::doubleParameter,
GestParam::stringParameter,
GestParam::vectorParameter>* GestEnv = NULL;

#ifdef QT_LIBRARY
GestParam* EnumGestureParameterObject = NULL;

GestParam::GestParam()
{

}

GestParam::~GestParam()
{

}
#endif

// @brief Function that inizializes the
// Parameter container
void initGestureParameters()
{
#ifdef QT_LIBRARY
  EnumGestureParameterObject = new GestParam;
#endif

        // Create 5 maps for all types and fill the 5 maps
        // ------------------------------------------------------------------
        std::map<GestParam::boolParameter,      boolContainer*>                  myBoolMap;
        std::map<GestParam::intParameter,       intContainer*>                   myIntMap;
        std::map<GestParam::doubleParameter,    doubleContainer*>                myDoubleMap;
        std::map<GestParam::stringParameter,    stringContainer*>                myStringMap;
        std::map<GestParam::vectorParameter,    vectorContainer*>                myVectorMap;

        // Bool
        // ------------------------------------------------------------------
        myBoolMap.insert( std::make_pair( GestParam::draw_robot_sampled_points,        new boolContainer(false)));
        myBoolMap.insert( std::make_pair( GestParam::draw_human_sampled_points,        new boolContainer(false)));
        myBoolMap.insert( std::make_pair( GestParam::draw_ws_occupancy,                new boolContainer(false)));
        myBoolMap.insert( std::make_pair( GestParam::draw_single_class,                new boolContainer(false)));


        // Int
        // ------------------------------------------------------------------
        myIntMap.insert(std::make_pair( GestParam::tata,                         new intContainer(5)));


        // Double
        // ------------------------------------------------------------------
        myDoubleMap.insert( std::make_pair( GestParam::tete,        new doubleContainer(10.0)));

    //cout << "PlanEnv->getDouble(p) = " << PlanEnv->getDouble( PlanParam::env_objectNessecity ) << endl;

        // String
        // ------------------------------------------------------------------
#ifdef QT_LIBRARY
        myStringMap.insert(std::make_pair(GestParam::titi,                       new stringContainer("titi")));
#endif

        // Vector
        // ------------------------------------------------------------------
        std::vector<double> tutu;
        tutu.push_back( 1 ); tutu.push_back( 8 );

        myVectorMap.insert(std::make_pair(GestParam::tutu,                       new vectorContainer(tutu)));

        // Make the new parameter container
        GestEnv =  new Parameters<
        GestParam::boolParameter,
        GestParam::intParameter,
        GestParam::doubleParameter,
        GestParam::stringParameter,
        GestParam::vectorParameter>(
                                                                                 myBoolMap,
                                                                                 myIntMap,
                                                                                 myDoubleMap,
                                                                                 myStringMap,
                                                                                 myVectorMap);
}
