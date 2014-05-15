/*
 *  HRI_costspace.hoo
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 27/04/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */
#ifndef HRI_COSTSPACE_HPP
#define HRI_COSTSPACE_HPP

#include "planner/cost_space.hpp"

#include "API/ConfigSpace/configuration.hpp"

#include <libmove3d/hri/hri.h>

// Main cost function
extern double HRICS_getConfigCost(Move3D::Configuration& Conf);
extern double HRICS_getPlanarHumanGridCost(Move3D::Configuration& q);
extern   void HRICS_init(HRI_AGENTS* agents = NULL);

// Human kinematics
const int HRICS_HUMANj_BODY=      2;
const int HRICS_HUMANj_NECK_PAN=  5;
const int HRICS_HUMANj_NECK_TILT= 6;
const int HRICS_HUMANj_RHAND=     29; /* or 30 or 31 */
const int HRICS_HUMANj_LHAND=     26; /* or 27 or 28 */

#include "HRICS_distance.hpp"
#include "HRICS_visibility.hpp"
#include "HRICS_natural.hpp"
#include "HRICS_workspace.hpp"
#include "HRICS_config_space.hpp"
#include "HRICS_otpmotionpl.hpp"
#include "HRICS_human_cost_space.hpp"
#include "HRICS_navigation.hpp"
#include "HRICS_legibility.hpp"

#ifdef HRI_PLANNER
#include "HRICS_hamp.hpp"
extern HRICS::HriSpaceCost* hriSpace;
#endif

// Elementary cost maps
const int HRICS_Distance = 0;
const int HRICS_Visibility = 1;
const int HRICS_Naturality = 2;
const int HRICS_Reachability = 3;
const int HRICS_Combine = 4;

/**
 * Active Elementary Cost Spaces 
 * Object
 */
extern HRICS::Distance*		HRICS_activeDist;
extern HRICS::Visibility*	HRICS_activeVisi;
extern HRICS::Natural*		HRICS_activeNatu;
extern HRICS::Natural*		HRICS_activeReac;
extern HRICS::Legibility*   HRICS_activeLegi;

extern HRICS::HumanCostSpace* HRICS_humanCostMaps;

/**
 * Active Motion planner framework
 */
extern HRICS::HumanAwareMotionPlanner*	HRICS_MotionPL;
extern HRICS::HumanAwareMotionPlanner*	HRICS_MotionPLConfig;

/**
 * Cells to be drawn
 */ 
extern Move3D::ThreeDCell*		BiasedCell3D;
extern Move3D::TwoDCell*		BiasedCell2D;

#endif

