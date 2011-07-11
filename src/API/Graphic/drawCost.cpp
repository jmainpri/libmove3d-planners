/*
 *  g3d_draw_cost.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 30/04/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */
#ifdef HRI_COSTSPACE
#include "HRI_costspace/HRICS_costspace.hpp"
#include "HRI_costspace/HRICS_otpmotionpl.hpp"
#include "HRI_costspace/HRICS_ConfigSpace.hpp"
#endif

#include "planner/planEnvironment.hpp"
#include "planner/Greedy/CollisionSpace.hpp"
#include "planner/TrajectoryOptim/trajectoryOptim.hpp"

#include "API/Grids/gridsAPI.hpp"

#include "P3d-pkg.h"
#include "Graphic-pkg.h"

#ifdef P3D_COLLISION_CHECKING
#include "Collision-pkg.h"
#endif

#include <Eigen/Core>
#define EIGEN_USE_NEW_STDVECTOR
#include <Eigen/StdVector>
#include <Eigen/Geometry> 

#include <iostream>
#include <tr1/memory>

using namespace std;
using namespace tr1;

Eigen::Vector3d global_DrawnSphere;
vector<Eigen::Vector3d> CXX_drawBox; 
vector<double> vect_jim;
double cost_max = 30.0;

extern Eigen::Vector3d current_WSPoint;
extern pair<double, Eigen::Vector3d > current_cost;
extern std::string hri_text_to_display;

extern std::vector<Eigen::Vector3d> OTPList;


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

void g3d_draw_squeleton()
{
  double boxA[8][3];
  double boxB[8][3];
  
  CXX_drawBox.resize(8);
  
  Robot* robot = global_Project->getActiveScene()->getRobotByNameContaining("PR2") ;
  
  for(unsigned int i=0; i<robot->getNumberOfJoints(); i++)
  {
    Joint* jnt = robot->getJoint(i);
    p3d_obj* obj = jnt->getJointStruct()->o;
    
    if(obj)
    {
      Eigen::Vector3d vect1,vect2,vect5,vect6;
      //pqp_draw_OBB_first_level(obj);
      
      if ( pqp_get_OBB_first_level(obj,boxA) )
      {
        for(int j=0; j<8; j++)
        {
          CXX_drawBox[j][0] = boxA[j][0];
          CXX_drawBox[j][1] = boxA[j][1]; 
          CXX_drawBox[j][2] = boxA[j][2];
          
          boxA[j][0] = CXX_drawBox[j][0];
          boxA[j][1] = CXX_drawBox[j][1];
          boxA[j][2] = CXX_drawBox[j][2];
          
//          if (j == 1) 
//          { vect1 = CXX_drawBox[j]; }
//          if (j == 2) 
//          { vect2 = CXX_drawBox[j]; }
//          if (j == 5) 
//          { vect5 = CXX_drawBox[j]; }
//          if (j == 6) 
//          { vect6 = CXX_drawBox[j]; }
          
          CXX_drawBox[j] = jnt->getMatrixPos()*CXX_drawBox[j];
          
          boxB[j][0] = CXX_drawBox[j][0];
          boxB[j][1] = CXX_drawBox[j][1];
          boxB[j][2] = CXX_drawBox[j][2];
          
//          if (j == ENV.getDouble(Env::extensionStep)) {
//            continue;
//          }
          
          double alpha = (double)j/8;
          double colorvector[4];
          GroundColorMixGreenToRed(colorvector,alpha);
          
//          glColor4dv(colorvector);
//          g3d_draw_simple_box(boxA[j][0],boxA[j][0]+0.02,
//                              boxA[j][1],boxA[j][1]+0.02,
//                              boxA[j][2],boxA[j][2]+0.02, FALSE, TRUE, 0.01);
          
          
          glColor4dv(colorvector);
          g3d_draw_simple_box(boxB[j][0],boxB[j][0]+0.02,
                              boxB[j][1],boxB[j][1]+0.02,
                              boxB[j][2],boxB[j][2]+0.02, FALSE, TRUE, 0.01);
        }
        
        //g3d_draw_complex_black_box(boxA);
        g3d_draw_complex_black_box(boxB);
        
        vect1 = CXX_drawBox[1];
        vect2 = CXX_drawBox[2];
        vect5 = CXX_drawBox[5];
        vect6 = CXX_drawBox[6];
        
        Eigen::Vector3d p1Eigen = 0.5*(vect1+vect2);
        Eigen::Vector3d p2Eigen = 0.5*(vect5+vect6);
        
        g3d_draw_simple_box(p1Eigen[0],p1Eigen[0]+0.02,
                            p1Eigen[1],p1Eigen[1]+0.02,
                            p1Eigen[2],p1Eigen[2]+0.02, FALSE, FALSE, 0.01);
        
        g3d_draw_simple_box(p2Eigen[0],p2Eigen[0]+0.02,
                            p2Eigen[1],p2Eigen[1]+0.02,
                            p2Eigen[2],p2Eigen[2]+0.02, FALSE, FALSE, 0.01);
        
        double p1[3], p2[3];
        double radius = 0.5*( ( vect1 - vect2 ).norm() );
        
        p1[0] = p1Eigen[0];
        p1[1] = p1Eigen[1];
        p1[2] = p1Eigen[2];
        
        p2[0] = p2Eigen[0];
        p2[1] = p2Eigen[1];
        p2[2] = p2Eigen[2];
        
        double colorvector[4];
        colorvector[0] = 1.0;
        colorvector[1] = 1.0;
        colorvector[2] = 0.0;
        colorvector[3] = 0.5;
        
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        
        glColor4dv(colorvector);
        g3d_draw_cylinder(p1, p2, radius, 20 );

        glDisable(GL_BLEND);
      }
    }
  }
}

void g3d_draw_grids()
{
  //g3d_draw_squeleton();
  //cout << "g3d_draw_grids" << endl;
  
  //pqp_draw_all_OBBs(1);
  
  //double tx, ty, tz, ax, ay, az;
  //double xmin, xmax, ymin, ymax, zmin, zmax;
  //p3d_matrix4 mat;

    
#ifdef HRI_COSTSPACE
  
	if( ENV.getBool(Env::drawEntireGrid) && API_activeGrid )
	{
		ENV.setBool(Env::drawGrid,true);
	}
  //-------------------------------------------------------------
	if( ENV.getBool(Env::drawGrid) && API_activeGrid )
	{
    //cout << "API_activeGrid->draw()" << endl;
    
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
	//-------------------------------------------------------------
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
	//-------------------------------------------------------------
	if( HRICS_activeNatu )
	{
    HRICS_activeNatu->printBodyPos();
	}

	if( API_activeRobotGrid && ENV.getBool(Env::DrawRobotBaseGridCosts) )
	{
		API_activeRobotGrid->draw();
	}
#endif
  
  if( global_CollisionSpace && ENV.getBool(Env::drawVectorField) ) 
  {
     global_CollisionSpace->drawGradient();  
  }
  if( global_CollisionSpace && PlanEnv->getBool(PlanParam::drawOccupVoxels) )
  {
    global_CollisionSpace->draw();
    //global_CollisionSpace->drawGradient();
  }
  
  if( global_CollisionSpace && PlanEnv->getBool(PlanParam::drawStaticVoxels) )
  {
    global_CollisionSpace->drawStaticVoxels();
  }
  
	if( global_CollisionSpace && PlanEnv->getBool(PlanParam::drawSampledPoints) )
  {
    global_CollisionSpace->getBodySampler()->draw();
  }
  
  if (global_CollisionSpace && PlanEnv->getBool(PlanParam::drawStaticVoxels)) 
  {
    global_CollisionSpace->drawSquaredDist();
  }
   
  if ( PlanEnv->getBool(PlanParam::drawBoundingVolumes) ) 
  {
    traj_optim_draw_collision_points();
  }
  
	if( ENV.getBool(Env::drawPoints) )
	{
		if(PointsToDraw != NULL)
		{
      //cout << "PointsToDraw = " << PointsToDraw << endl;
			PointsToDraw->drawAllPoints();
		}
	}
	
	// Draws a sphere of 10 cm of radius
  g3d_drawSphere(global_DrawnSphere(0),
  							 global_DrawnSphere(1),
  							 global_DrawnSphere(2), 0.02 );
}

void drawGauge(int number, double cost)
{
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  
  double cylinderColorvector[4];
  
  cylinderColorvector[1] = 1.0;       //green
  cylinderColorvector[2] = 0.0;       //blue
  cylinderColorvector[0] = 0.0;       //red
  cylinderColorvector[3] = 0.7;       //transparency
  
  GroundColorMixGreenToRed(cylinderColorvector, cost);
  
  g3d_set_color(Any,cylinderColorvector);
  
  
  g3d_draw_solid_cylinder(1.0, -1.0 - (number * 0.5), 0.09, cost * 2, 30);
  
  cylinderColorvector[1] = 0.5;       //green
  cylinderColorvector[2] = 0.5;       //blue
  cylinderColorvector[0] = 0.5;       //red
  cylinderColorvector[3] = 0.4;       //transparency
  
  g3d_set_color(Any,cylinderColorvector);
  g3d_draw_solid_cylinder(1.0, -1.0 - (number * 0.5), 0.10, 2, 30);
  
  glDisable(GL_BLEND);
}

void drawSlice();

/**
 * @ingroup graphics
 * Draws the thing related to HRI_COSTSPACE
 */
#ifdef HRI_COSTSPACE
void g3d_draw_hrics()
{
    int OTPListSize = OTPList.size();
//    if (OTPListSize > 0)
//    {
//        drawSlice();
//    }
//    return;

	if( ENV.getBool(Env::enableHri) )
	{
		if( ENV.getBool(Env::HRIPlannerCS) && ENV.getBool(Env::drawTraj) )
		{
			//          printf("Draw 2d path\n");
			//dynamic_cast<HRICS::ConfigSpace*>(HRICS_MotionPLConfig)->draw2dPath();
		}

		if( ENV.getBool(Env::drawOTPTraj) )
		{
			dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->draw2dPath();
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
  
//  if( HRICS_activeDist )
//	{
//		HRICS_activeDist->drawInteractionZone();
//  }
//  
//  if (HRICS_MotionPL) {
//    dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->drawCurrentOTP();
//  }
  
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
    
    if( HRICS_activeNatu && PlanEnv->getBool(PlanParam::drawColorConfig) )
    {
      HRICS_activeNatu->setRobotColorFromConfiguration(true);
    }
	}
  
	if ( ENV.getBool(Env::drawGaze) && ( ENV.getBool(Env::HRIPlannerWS) ||  ENV.getBool(Env::HRIPlannerCS) ) )
	{
		vector<double> Gaze;
		Gaze.clear();
		
		//cout << "Draw Gaze" << endl;
		
		Gaze = HRICS_MotionPL->getVisibility()->getGaze();
		
    //		glLineWidth(3.);
    //		
    //		if( (Gaze.size() == 6))
    //		{		
    //			g3d_drawOneLine(Gaze[0], Gaze[1],
    //											Gaze[2], Gaze[3],
    //											Gaze[4], Gaze[5], Blue, NULL);
    //		}
    //		
    //		glLineWidth(1.);
    
    GLdouble GreenColor[4] =   { 0.0, 0.5, 0.0, 0.7 };
    GLdouble GreenColorT[4] =   { 0.0, 0.5, 0.0, 0.0 };
//    GLdouble GreyColor[4] =   { 0.5, 0.5, 0.5, 0.5 };
//    GLdouble GreyColorT[4] =   { 0.5, 0.5, 0.5, 0.0 };


    Robot* human = dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->getHuman();
    int gazeIndex = 46;
    if ( human->getName() == "ACHILE_HUMAN1" )
    {
        gazeIndex = 42;
    }

//    cout << "HUMAN = " << human->getName() << endl;
    p3d_jnt* eyes = human->getJoint(gazeIndex)->getJointStruct();

    // 46 is for HERAKLES
    // 42 is for ACHILE
    g3d_draw_visibility_by_frame(eyes->abs_pos,DTOR(160),DTOR(160*0.75),1, GreenColor, GreenColorT);

	}

	if (PlanEnv->getBool(PlanParam::drawRandomMap))
	{
		double depth = 1.0;

		shared_ptr<Configuration> q_hum = dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->getHuman()->getCurrentPos();
		int indexFirstDof = dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->getHuman()->getJoint("Pelvis")->getIndexOfFirstDof();

		double xMin = (*q_hum)[indexFirstDof + 0] + PlanEnv->getDouble(PlanParam::env_randomXMinLimit);
		double xMax = (*q_hum)[indexFirstDof + 0] + PlanEnv->getDouble(PlanParam::env_randomXMaxLimit);
		double yMin = (*q_hum)[indexFirstDof + 1] + PlanEnv->getDouble(PlanParam::env_randomYMinLimit);
		double yMax = (*q_hum)[indexFirstDof + 1] + PlanEnv->getDouble(PlanParam::env_randomYMaxLimit);
		glLineWidth(3.);
		g3d_drawOneLine(xMin,   yMin,    depth,
						xMin,   yMax,    depth,
						Black, NULL);
		glLineWidth(1.);

		glLineWidth(3.);
		g3d_drawOneLine(xMin,   yMax,    depth,
						xMax,   yMax,    depth,
						Black, NULL);
		glLineWidth(1.);

		glLineWidth(3.);
		g3d_drawOneLine(xMax,   yMax,    depth,
						xMax,   yMin,    depth,
						Black, NULL);
		glLineWidth(1.);

		glLineWidth(3.);
		g3d_drawOneLine(xMax,   yMin,    depth,
						xMin,   yMin,    depth,
						Black, NULL);
		glLineWidth(1.);
	}
  
  if( HRICS_MotionPL )
	{
//    bool rightHand = true;
//    Eigen::Vector3d WSPoint = dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->computeOTPFromHandPose( rightHand );
//    
//    double color_array[4];
//    
//    glEnable(GL_BLEND);
//    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//    
//    color_array[0]= 0.0;
//    color_array[1]= 0.0;
//    color_array[2]= 1.0;
//    color_array[3]= 0.7;
//    g3d_set_color(Any,color_array);
//    g3d_draw_solid_sphere(WSPoint[0], WSPoint[1], WSPoint[2], 0.10, 10);
//    
//    glDisable(GL_BLEND);
  }
  
  
  //"HRI cost = %2.2f"
  
  if (current_WSPoint(0) != 0 && current_WSPoint(1) != 0)
  {


    double colorvector[4];
    
    colorvector[1] = 1.0;       //green
    colorvector[2] = 0.0;       //blue
    
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    //                Vector3d center = getWorkspacePoint();
    
    colorvector[0] = 0.0;       //red
    colorvector[3] = 0.9;       //transparency
    
    g3d_set_color(Any,colorvector);
    g3d_draw_solid_sphere(current_WSPoint[0], current_WSPoint[1], current_WSPoint[2], 0.02, 10);
    //                        GroundColorMixGreenToRed(colorvector,Cost);
    
    colorvector[0] = 1.0;       //red
    colorvector[3] = 0.5;       //transparency
    
    g3d_set_color(Any,colorvector);
    g3d_draw_solid_sphere(current_WSPoint[0], current_WSPoint[1], current_WSPoint[2], 0.10, 30);
    
    //    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);
//    drawGauge(0, current_cost.first/cost_max);
    
//    drawGauge(1, current_cost.second[0]);
//    drawGauge(2, current_cost.second[1]);
//    drawGauge(3, current_cost.second[2]);

    shared_ptr<Configuration> q_rob = dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->getRobot()->getCurrentPos();
    shared_ptr<Configuration> q_hum = dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->getHuman()->getCurrentPos();

//    glLineWidth(3.);
//    g3d_drawOneLine((*q_hum)[indexFirstDof + 0],   (*q_hum)[indexFirstDof + 1],    current_WSPoint[2],
//                    (*q_rob)[indexFirstDof + 0],   (*q_rob)[indexFirstDof + 1],    current_WSPoint[2],
//                    Yellow, NULL);
//    glLineWidth(1.);


  }
  OTPListSize = OTPList.size();
  if (OTPListSize > 0)
  {
      drawSlice();
      shared_ptr<Configuration> q_hum = dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->getHuman()->getCurrentPos();
      int indexFirstDof = dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->getHuman()->getJoint("Pelvis")->getIndexOfFirstDof();
      for (unsigned int i=0; i < OTPList.size(); i ++)
      {
          double colorvector[4];
          colorvector[0] = 0.0;       //red
          colorvector[1] = 1.0;       //green
          colorvector[2] = 0.0;       //blue
          colorvector[3] = 0.9;       //transparency

          glEnable(GL_BLEND);
          g3d_set_color(Any,colorvector);
          double pDist = std::sqrt(pow( OTPList.at(i)[0], 2) + pow( OTPList.at(i)[1], 2));
          double ang = atan2(OTPList.at(i)[1],OTPList.at(i)[0]);
          g3d_draw_solid_sphere(cos(ang + (*q_hum)[indexFirstDof + 5])*pDist + (*q_hum)[indexFirstDof + 0],
                                sin(ang + (*q_hum)[indexFirstDof + 5])*pDist + (*q_hum)[indexFirstDof + 1],
                                OTPList.at(i)[2] + (*q_hum)[indexFirstDof + 2],
                                0.02, 10);
          glDisable(GL_BLEND);
      }
  }


}
#endif


void computeConfigCostOnTraj(p3d_rob* rob,configPt q)
{
  if(ENV.getBool(Env::isCostSpace))
  {
    p3d_rob* costRobot = rob;
    configPt cost_q = q;
    
#ifdef HRI_COSTSPACE
    if ( ENV.getBool(Env::enableHri) ) 
    {
      std::string robotName(costRobot->name);
      
      if( robotName.find( global_ActiveRobotName ) == std::string::npos ) // Does not contain Robot
      {
        costRobot = p3d_get_robot_by_name_containing( global_ActiveRobotName.c_str() );
        cost_q = p3d_get_robot_config(costRobot);
      }
    }
#endif
    
    Robot* r_Cost( global_Project->getActiveScene()->getRobotByName(costRobot->name) );
    Configuration	q_Cost(r_Cost,cost_q);
    
    std::cout << "Cost for " << r_Cost->getName() << " = " 
    << global_costSpace->cost(q_Cost) << std::endl;
  }
}

void drawSlice()
{
    cout << "Draw slice" << endl;
    Robot* rob = global_Project->getActiveScene()->getRobotByNameContaining("PR2");
    Robot* human = dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->getHuman();
    shared_ptr<Configuration> q_cur(human->getCurrentPos());
//    getConfListSize() {return m_configList.size();}
//            configPt getRobotConfigAt

    if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig))
    {
        dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getConfList();
    }

    G3D_Window *win;
    win = g3d_get_cur_win();
    win->vs.transparency_mode= G3D_TRANSPARENT_AND_OPAQUE;
    win->vs.transparency_mode= G3D_OPAQUE;

    for(int i = 0; i < dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getConfListSize(); i++)
    {
//        shared_ptr<Configuration> q( new Configuration(rob,dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getRobotConfigAt(i) ));
//        rob->setAndUpdate(*q);
        dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->setRobotsToConf(i,PlanEnv->getBool(PlanParam::env_isStanding));
        global_Project->getActiveScene()->setActiveRobot("PR2_ROBOT");
        //cout << "g3d_draw_robot" << endl;
        //q->print();
        g3d_draw_robot(rob->getRobotStruct()->num, win );

//        shared_ptr<Configuration> q( new Configuration(human,dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getRobotConfigAt(i) ));
//        human->setAndUpdate(*q);
//        global_Project->getActiveScene()->setActiveRobot("ACHILE_HUMAN1");
//        //cout << "g3d_draw_robot" << endl;
//        //q->print();
//        g3d_draw_robot(human->getRobotStruct()->num, win);
    }
    human->setAndUpdate(*q_cur);
    cout << "--------------------------" << endl;
}
