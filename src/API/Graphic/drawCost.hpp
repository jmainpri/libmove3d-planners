/*
 *  g3d_draw_cost.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 30/04/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#ifndef G3D_DRAW_COST_H
#define G3D_DRAW_COST_H

#include "device.h"


void g3d_draw_costspace();

void g3d_draw_grids();


#ifdef HRI_COSTSPACE
void g3d_draw_hrics();
#endif

void drawGauge(int number, double cost);

void computeConfigCostOnTraj(p3d_rob* rob,configPt q);

#endif
