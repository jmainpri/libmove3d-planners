/*
 *  g3d_draw_cost.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 30/04/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "P3d-pkg.h"
#include "Graphic-pkg.h"

#ifdef P3D_COLLISION_CHECKING
#include "Collision-pkg.h"
#endif

#ifdef HRI_COSTSPACE
#include "HRI_costspace/HRICS_costspace.hpp"
#endif

#include "API/Grids/gridsAPI.hpp"

#include <Eigen/Core>
#define EIGEN_USE_NEW_STDVECTOR
#include <Eigen/StdVector>
#include <Eigen/Geometry> 
std::vector<Eigen::Vector3d> CXX_drawBox; 
Eigen::Vector3d global_DrawnSphere;

#include <iostream>
#include <tr1/memory>

using namespace std;
using namespace tr1;

std::vector<double> vect_jim;

// TODO callback OOMOVE3D
//#if defined( CXX_PLANNER )

//! @ingroup graphic
//! Function drawing a box the V7 is bellow V5
//  
//     V5 -- V6
//    /      / |
//   V1 -- V2 V8
//   |      | /
//   V3 -- V4
//
void g3d_draw_eigen_box(	const Eigen::Vector3d& v1, const Eigen::Vector3d& v2, const Eigen::Vector3d& v3, const Eigen::Vector3d& v4,
							const Eigen::Vector3d& v5, const Eigen::Vector3d& v6, const Eigen::Vector3d& v7, const Eigen::Vector3d& v8,
						int color, int fill, double width) {
	
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	if(fill == 0) {
		/* filaire */
		g3d_set_color(color, NULL);
	}
	
	glPushAttrib(GL_LINE_BIT);
	glLineWidth(width);
	glBegin(GL_LINES);
	glVertex3d(v1[0],v1[1],v1[2]);
	glVertex3d(v3[0],v3[1],v3[2]);
	
	glVertex3d(v2[0],v2[1],v2[2]);
	glVertex3d(v4[0],v4[1],v4[2]);
	
	glVertex3d(v5[0],v5[1],v5[2]);
	glVertex3d(v7[0],v7[1],v7[2]);
	
	glVertex3d(v6[0],v6[1],v6[2]);
	glVertex3d(v8[0],v8[1],v8[2]);
	glEnd();
	
	glBegin(GL_LINE_LOOP);
	glVertex3d(v1[0],v1[1],v1[2]);
	glVertex3d(v2[0],v2[1],v2[2]);
	glVertex3d(v6[0],v6[1],v6[2]);
	glVertex3d(v5[0],v5[1],v5[2]);
	glEnd();
	
	glBegin(GL_LINE_LOOP);
	glVertex3d(v3[0],v3[1],v3[2]);
	glVertex3d(v4[0],v4[1],v4[2]);
	glVertex3d(v8[0],v8[1],v8[2]);
	glVertex3d(v7[0],v7[1],v7[2]);
	glEnd();
	
	glPopAttrib();
}
//#endif

/**
 * @ingroup graphics
 * Draws the things related to cost spaces
 */
void g3d_draw_costspace()
{
	if( ENV.getBool(Env::isCostSpace) )
	{
		for (int num = 0; num < 2; num++)
		{
			for (int it = 0; it < 3; it++)
			{
#ifdef P3D_COLLISION_CHECKING
				if (vectMinDist[num][it] != 0)
				{
					g3d_drawOneLine(vectMinDist[0][0], vectMinDist[0][1], 
													vectMinDist[0][2], vectMinDist[1][0], 
													vectMinDist[1][1], vectMinDist[1][2], Red, NULL);
					break;
				}
#endif
			}
		}
	}
}

void g3d_draw_grids()
{
#ifdef HRI_COSTSPACE
	if( ENV.getBool(Env::drawGrid) && API_activeGrid )
	{
		API_activeGrid->draw();
		
		if (ENV.getBool(Env::drawBox)) 
		{
			CXX_drawBox = API_activeGrid->getBox();
		
			if (!CXX_drawBox.empty()) 
			{
				g3d_draw_eigen_box(	CXX_drawBox[0], CXX_drawBox[1], CXX_drawBox[2], CXX_drawBox[3],
														CXX_drawBox[4], CXX_drawBox[5], CXX_drawBox[6], CXX_drawBox[7],
													 Red, 0, 3);
			}
		}
	}
	
	if (ENV.getBool(Env::drawBox)) 
	{
		CXX_drawBox = API_activeRobot->getObjectBox();
		
		if (!CXX_drawBox.empty()) 
		{
			g3d_draw_eigen_box(	CXX_drawBox[0], CXX_drawBox[1], CXX_drawBox[2], CXX_drawBox[3],
													CXX_drawBox[4], CXX_drawBox[5], CXX_drawBox[6], CXX_drawBox[7],
													Red, 0, 3);
		}
	}
	
	if( HRICS_activeNatu )
	{
	   HRICS_activeNatu->printBodyPos();
	}
	
#endif
	
// TODO callback OOMOVE3D
#if defined( CXX_PLANNER )
	if( ENV.getBool(Env::drawPoints) )
	{
		if(PointsToDraw != NULL)
		{
			//cout << "Drawing points" << endl;
			PointsToDraw->drawAllPoints();
		}
	}
	
	// Draws a sphere of 10 cm of radius
//	g3d_drawSphere(global_DrawnSphere(0),
//								 global_DrawnSphere(1),
//								 global_DrawnSphere(2), 0.1 );
#endif
}

/**
 * @ingroup graphics
 * Draws the thing related to HRI_COSTSPACE
 */
#ifdef HRI_COSTSPACE
void g3d_draw_hrics()
{
	if( ENV.getBool(Env::enableHri) )
	{
		if( ENV.getBool(Env::HRIPlannerCS) && ENV.getBool(Env::drawTraj) )
		{
			//          printf("Draw 2d path\n");
			dynamic_cast<HRICS::ConfigSpace*>(HRICS_MotionPL)->draw2dPath();
		}
		
		if( ENV.getBool(Env::isCostSpace) )
		{
			if( ENV.getBool(Env::enableHri) )
			{
				if( ENV.getBool(Env::HRIPlannerWS) && ENV.getBool(Env::drawTraj) )
				{
					//              printf("Draw 3d path\n");
					dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->draw3dPath();
				}
			}
		}
	}
	
	if( ENV.getBool(Env::drawDistance) && HRICS_activeDist )
	{
		vect_jim = HRICS_activeDist->getVectorJim();
		
		glLineWidth(3.);
		
		for (unsigned int i = 0; i < vect_jim.size() / 6; i++)
		{
			g3d_drawOneLine(vect_jim[0 + 6 * i], vect_jim[1 + 6 * i],
											vect_jim[2 + 6 * i], vect_jim[3 + 6 * i],
											vect_jim[4 + 6 * i], vect_jim[5 + 6 * i], Red, NULL);
		}
		
		glLineWidth(1.);
	}

	if ( ENV.getBool(Env::drawGaze) && ( ENV.getBool(Env::HRIPlannerWS) ||  ENV.getBool(Env::HRIPlannerCS) ) )
	{
		vector<double> Gaze;
		Gaze.clear();
		
		//cout << "Draw Gaze" << endl;
		
		Gaze = HRICS_MotionPL->getVisibility()->getGaze();
		
		glLineWidth(3.);
		
		if( (Gaze.size() == 6))
		{		
			g3d_drawOneLine(Gaze[0], Gaze[1],
											Gaze[2], Gaze[3],
											Gaze[4], Gaze[5], Blue, NULL);
		}
		
		glLineWidth(1.);
	}
}
#endif

