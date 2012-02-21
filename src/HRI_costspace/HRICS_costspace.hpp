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
extern double HRICS_getConfigCost(Configuration& Conf);
extern   void HRICS_init(HRI_AGENTS* agents = NULL);

// Human kinematics
const int HRICS_HUMANj_BODY=      2;
const int HRICS_HUMANj_NECK_PAN=  5;
const int HRICS_HUMANj_NECK_TILT= 6;
const int HRICS_HUMANj_RHAND=     29; /* or 30 or 31 */
const int HRICS_HUMANj_LHAND=     26; /* or 27 or 28 */

#include "HRICS_Distance.hpp"
#include "HRICS_Visibility.hpp"
#include "HRICS_Natural.hpp"
#include "HRICS_Workspace.hpp"
#include "HRICS_ConfigSpace.hpp"
#include "HRICS_otpmotionpl.hpp"
#include "HRICS_humanCostSpace.hpp"

#ifdef HRI_PLANNER
#include "HRICS_HAMP.hpp"
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

extern HRICS::HumanCostSpace* HRICS_humanCostMaps;

/**
 * Active Motion planner framework
 */
extern HRICS::HumanAwareMotionPlanner*	HRICS_MotionPL;
extern HRICS::HumanAwareMotionPlanner*	HRICS_MotionPLConfig;

/**
 * Cells to be drawn
 */ 
extern API::ThreeDCell*		BiasedCell3D;
extern API::TwoDCell*		  BiasedCell2D;

#endif

